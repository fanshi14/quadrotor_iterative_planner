
#include <iterative_planner_server/IterativePlanner.h>

void IterativePlanner::onInit()
{

  ros::NodeHandle private_nh("~");

  private_nh.param("collision_detection", m_collision_detection_flag, false);
  private_nh.param("dji_mode", m_dji_mode, false);
  private_nh.param("debug_mode", m_debug_mode, false);
  private_nh.param("landing_mode", m_landing_mode, false);
  private_nh.param("gazebo_mode", m_gazebo_mode, false);
  private_nh.param("relative_traj_track_mode", m_relative_traj_track_mode, false);

  private_nh.param("resolution", m_octomap_res, 0.1);
  private_nh.param("tree_depth", m_octomap_tree_depth, 16);
  private_nh.param("route_id", m_route_id, 1);
  private_nh.param("spline_degree", m_spline_degree, 2);
  private_nh.param("vehicles_visualize_period_time", m_vehicles_visualize_period_time, 0.5);
  private_nh.param("global_planning_period_time", m_global_planning_period_default_time, 1.0);
  private_nh.param("target_height", m_target_height, 0.8);
  private_nh.param("uav_default_upbound_vel", m_uav_default_upbound_vel, 7.0);
  private_nh.param("uav_landing_time_xy_upbound", m_uav_landing_time_xy_upbound, 5.0);
  private_nh.param("uav_default_tracking_time", m_uav_default_tracking_time, 2.0);
  private_nh.param("target_odom_sub_topic_name", m_target_odom_sub_topic_name, (std::string)"/truck_odom");
  private_nh.param("spline_path_pub_topic_name", m_spline_path_pub_topic_name, (std::string)"spline_path");
  private_nh.param("uav_odom_sub_topic_name", m_uav_odom_sub_topic_name, (std::string)"/ground_truth/state");
  private_nh.param("uav_cmd_pub_topic_name", m_uav_cmd_pub_topic_name, (std::string)"/cmd_vel");
  private_nh.param("uav_direct_start_mode", m_uav_direct_start_mode, true);
  private_nh.param("target_traj_deviation_threshold", m_target_traj_deviation_threshold, 1.0);
  private_nh.param("trajectory_function_print_flag", m_target_traj_param_print_flag, true);
  private_nh.param("uav_landing_constant_speed", m_uav_landing_constant_vel, -0.4);
  private_nh.param("uav_force_landing_speed", m_uav_force_landing_vel, -1.0);
  private_nh.param("uav_force_landing_method", m_uav_force_landing_method, 1);
  /* restricted region */
  private_nh.param("restricted_region_mode", m_restricted_region_mode, true);
  private_nh.param("restricted_region_radius", m_restricted_region_radius, 48.24);

  /* Init */
  m_uav.onInit();
  m_bspline_generator.onInit(m_spline_degree, true, m_spline_path_pub_topic_name);
  if (m_collision_detection_flag || m_gazebo_mode){
    m_target_ptr = new VehicleOctomapServer(m_octomap_res, m_octomap_tree_depth);
  }
  m_octomap_boarder_val = m_octomap_res * pow(2, m_octomap_tree_depth-1);
  m_vehicles_visualize_prev_time = ros::Time().now().toSec();
  m_vehicle_traj_recv_time = m_vehicles_visualize_prev_time;
  m_global_planning_period_time = m_global_planning_period_default_time;
  if (m_uav_direct_start_mode)
    m_uav.m_uav_state = 3; // in direct start mode, uav is ready to directly tracking.
  m_restricted_region_recv_flag = false;
  m_uav_force_landing_start_time = -1;

  /* Subscriber */
  m_sub_lane_marker_flag = m_nh.subscribe<std_msgs::Empty>("/lane_marker_flag", 1, &IterativePlanner::laneMarkerCallback, this);
  m_sub_target_traj_param = m_nh.subscribe<quadrotor_trajectory::TrackParamStamped>("/truck_traj_param", 1, &IterativePlanner::targetTrajParamCallback, this);
  m_sub_target_odom = m_nh.subscribe<nav_msgs::Odometry>(m_target_odom_sub_topic_name, 1, &IterativePlanner::targetOdomCallback, this);
  m_sub_uav_odom = m_nh.subscribe<nav_msgs::Odometry>(m_uav_odom_sub_topic_name, 1, &IterativePlanner::uavOdomCallback, this);
  m_sub_uav_start_flag = m_nh.subscribe<std_msgs::Empty>("/simulator_uav_start_flag", 1, &IterativePlanner::uavStartFlagCallback, this);
  m_sub_uav_straight_lane_landing_start_flag = m_nh.subscribe<std_msgs::Empty>("/uav_straight_lane_landing_start_flag", 1, &IterativePlanner::uavStraightLaneLandingStartFlagCallback, this);
  m_sub_restricted_region_center = m_nh.subscribe<nav_msgs::Odometry>("/restricted_region_center", 1, &IterativePlanner::restrictedRegionCenterCallback, this);
  m_sub_uav_arrive_gps_point_flag = m_nh.subscribe<std_msgs::Empty>("/task1_arrive_gps_point", 1, &IterativePlanner::uavArriveGpsPointFlagCallback, this);


  /* Publisher */
  m_pub_reconstructed_path_markers = m_nh.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);
  m_pub_uav_cmd  = m_nh.advertise<geometry_msgs::Twist>(m_uav_cmd_pub_topic_name, 1);
  m_pub_estimated_target_odom = m_nh.advertise<nav_msgs::Odometry>("truck_odom_estimated", 1);


  ROS_INFO("onInit finished");
}

void IterativePlanner::initIterativeSearching()
{
  /* Clear previous vector data, marker, octree data */
  if (!m_control_point_vec.empty()){
    controlPolygonDisplay(0);
    m_control_point_vec.clear();
  }

  if (m_collision_detection_flag){
    if (!m_object_seg_ptr_vec.empty()){
      for (int i = 0; i < m_object_seg_ptr_vec.size(); ++i)
        m_object_seg_ptr_vec[i]->m_octree->clear();
      m_object_seg_ptr_vec.clear();
    }
  }

  /* Initialize segment estimated landing time, which could be dynamically adjusted when meeting obstacles */
  // Curently we do not have data in truck odometry`s twist.
  // double truck_vel = sqrt(pow(m_target_odom.twist.twist.linear.x, 2) +pow(m_target_odom.twist.twist.linear.y, 2));
  double target_vel = 4.17;
  double landing_time_xy = sqrt(pow(m_uav_odom.pose.pose.position.x-m_target_odom.pose.pose.position.x, 2) + pow(m_uav_odom.pose.pose.position.y-m_target_odom.pose.pose.position.y, 2)) / (m_uav_default_upbound_vel - target_vel);
  m_landing_time = landing_time_xy;
  if (m_landing_mode){
    /* If not start force landing, do not consider the influence from height to landing time, */
    if (m_uav.m_uav_state == 7){ // for method 2, generate last trajectory to follow
      double landing_time_z = (m_uav_odom.pose.pose.position.z - m_target_height) / abs(m_uav_force_landing_vel/2.0);
      if (landing_time_z > landing_time_xy)
        m_landing_time = landing_time_z;
      else
        m_landing_time = landing_time_xy;
    }
  }
  m_n_total_segments = (int)(m_landing_time + 0.49);
  if (m_landing_time > m_uav_landing_time_xy_upbound){
    m_landing_time = m_uav_landing_time_xy_upbound;
    m_n_segments = (int)(m_landing_time + 0.49);
  }
  /* If landing time is less than 2.0s, then assume its bspline has 2 segments. Because if each trajectory period time is too short, will make uav movment very unstable */
  else if (m_landing_time <= 2.0){
    if (m_uav.m_uav_state == 7){ // method 2, if last trajectory to follow is too short, it is also okay
      // if (m_landing_time < 0.5) // make last trajectory at least 0.5s
      //   m_landing_time = 0.5;
    }
    else{
      // m_landing_time = 2.0;
      if (m_landing_time < 2 * m_global_planning_period_default_time)
        m_landing_time = 2 * m_global_planning_period_default_time;
    }

    m_n_total_segments = 2;
    m_n_segments = 2;
  }
  else
    m_n_segments = (int)(m_landing_time + 0.49);

  /* If landing time is too small, make planning time longer */
  if (m_landing_time < m_global_planning_period_time)
    m_global_planning_period_time = m_landing_time;

  /* Initialize segment default period time, which is 1.0s */
  m_segment_period_time = m_landing_time / m_n_segments;

  Vector3d control_pt_0(m_uav_odom.pose.pose.position.x, m_uav_odom.pose.pose.position.y, m_uav_odom.pose.pose.position.z);
  m_control_point_vec.push_back(control_pt_0);
  // Set second control point: P1 = P0 + v * Tp / 2
  while(1){
    if (m_collision_detection_flag){
      VehicleOctomapServer* obstacle_ptr = new VehicleOctomapServer(m_octomap_res, m_octomap_tree_depth, false);
      /* Add obstacle of inner and outter cars, and static obstacles like bridge */
      // updateObstacleOctomap(obstacle_ptr, 0);
      // todo: collision detection
      m_object_seg_ptr_vec.push_back(obstacle_ptr);
    }
    Vector3d control_pt_1 = control_pt_0 +
      Vector3d(m_uav_odom.twist.twist.linear.x*m_segment_period_time/2.0,
               m_uav_odom.twist.twist.linear.y*m_segment_period_time/2.0,
               m_uav_odom.twist.twist.linear.z*m_segment_period_time/2.0);
    if (m_restricted_region_mode && m_restricted_region_recv_flag)
      restrictedControlPoint(control_pt_1);
    m_control_point_vec.push_back(control_pt_1);

    break;
  }
}

void IterativePlanner::onIterativeSearching()
{
  /* Simple generate control points, ignoring obstacles avoidance */
  Vector3d target_cur_pt;
  if (m_landing_mode)
    target_cur_pt = Vector3d(m_target_odom.pose.pose.position.x, m_target_odom.pose.pose.position.y, m_target_height);
  else
    target_cur_pt = Vector3d(m_target_odom.pose.pose.position.x, m_target_odom.pose.pose.position.y, (m_control_point_vec[0])[2]);
  for (int i = 1; i <= m_n_segments-2; ++i){
    Vector3d cur_control_pt;
    cur_control_pt = (m_control_point_vec[0] - target_cur_pt) / (m_n_total_segments-1) * (m_n_total_segments-1-i) + m_target_traj_base.nOrderVehicleTrajectory(0, i*m_segment_period_time) + Vector3d(0.0, 0.0, target_cur_pt[2]);
    if (m_restricted_region_mode && m_restricted_region_recv_flag)
      restrictedControlPoint(cur_control_pt);
    m_control_point_vec.push_back(cur_control_pt);
  }

  /* Set second control point: Pn-1 = Pn - v * Tp / 2 */
  /* Simple add last 2 control points without collision checking */
  // todo: check last 2 control points
  while(1){
    Vector3d control_pt_n;
    Vector3d control_pt_n_1;
    if (m_landing_mode)
      control_pt_n = m_target_traj_base.nOrderVehicleTrajectory(0, m_landing_time) + Vector3d(0.0, 0.0, m_target_height);
    else
      control_pt_n = m_target_traj_base.nOrderVehicleTrajectory(0, m_landing_time) + Vector3d(0.0, 0.0, (m_control_point_vec[0])[2]);
    control_pt_n_1 = control_pt_n - m_target_traj_base.nOrderVehicleTrajectory(1, m_landing_time) * m_segment_period_time/2.0;
    control_pt_n_1[2] = control_pt_n[2] + abs(m_uav_landing_constant_vel) * m_segment_period_time/2.0;
    if (m_uav.m_uav_state == 7){ // method 2, last point speed in z axis equal to force landing speed.
      control_pt_n_1[2] = m_target_height + abs(m_uav_force_landing_vel) * m_segment_period_time/2.0;
      /* force landing trajectory plan finished, change to during force land mode */
      m_uav.m_uav_state = 8; // state 8: during force land
    }
    if (m_collision_detection_flag){
      VehicleOctomapServer* obstacle_ptr = new VehicleOctomapServer(m_octomap_res, m_octomap_tree_depth, false);
      /* Add obstacle of inner and outter cars, and static obstacles like bridge */
      // updateObstacleOctomap(obstacle_ptr, m_landing_time-m_segment_period_time);
      Vector3d temp_3d;
      /* Check control points whether is safe */
      // todo: stretegy when it happens
      if (getGridCenter(obstacle_ptr, control_pt_n, temp_3d, -1)){
        ROS_ERROR("Landing point is in collision!!!");
      }
      /* Check landing point's neighbor control point being safe */
      // todo: strategy when this point is in collision
      if (!getGridCenter(obstacle_ptr, control_pt_n_1, temp_3d, -1)){
        ROS_WARN("Control points n-1 could not use!! TODO work.");
      }
    }
    if (m_restricted_region_mode && m_restricted_region_recv_flag){
      restrictedControlPoint(control_pt_n_1);
      restrictedControlPoint(control_pt_n);
    }
    m_control_point_vec.push_back(control_pt_n_1);
    m_control_point_vec.push_back(control_pt_n);
    break;
  }
}

void IterativePlanner::runIterativeSearching()
{
  // test
  /* uav odom cheat mode */
  // uavRandomCheatOdom();

  initIterativeSearching();
  onIterativeSearching();

  /* Input control points as param into bspline member */
  geometry_msgs::PolygonStamped control_polygon_points;
  for (int i = 0; i < m_control_point_vec.size(); ++i){
    geometry_msgs::Point32 control_point, time_point;
    time_point.x = m_segment_period_time * i;
    control_polygon_points.polygon.points.push_back(time_point);
    vector3dConvertToPoint32(m_control_point_vec[i], control_point);
    control_polygon_points.polygon.points.push_back(control_point);
  }
  m_bspline_generator.bsplineParamInput(&control_polygon_points);
  m_bspline_generator.getDerive();

  /* test: specific control points test */
  //controlPtsRandomSet();

  /* publish control points */
  controlPolygonDisplay(1);

  //ROS_INFO("Iterative searching is finished.");

  /* Print the value of control points */
  // for (int i = 0; i < m_control_point_vec.size(); ++i){
  //   std::cout << "[Control pt] " << i << ": " << m_control_point_vec[i].x() << ", "
  //             << m_control_point_vec[i].y() << ", " << m_control_point_vec[i].z() << "\n";
  // }
}

void IterativePlanner::uavRandomCheatOdom()
{
  m_uav_odom = m_target_odom;
  // position: target behind 4.0m, height is 5m
  m_uav_odom.pose.pose.position.z = 4;
  m_uav_odom.pose.pose.position.x += 1.0 + (-3.0) * cos(m_uav_odom.pose.pose.orientation.w);
  m_uav_odom.pose.pose.position.y += 1.0 + (-3.0) * sin(m_uav_odom.pose.pose.orientation.w);
  // velocity: 0, 0, 0
  m_uav_odom.twist.twist.linear.x = 5.0;
  m_uav_odom.twist.twist.linear.y = 5.0;
  m_uav_odom.twist.twist.linear.z = 0.0;
}

void IterativePlanner::controlPtsRandomSet()
{
  /* test: specific control points test */
  if (!m_control_point_vec.empty()){
    controlPolygonDisplay(0);
    m_control_point_vec.clear();
  }
  m_segment_period_time = 1.0;
  Vector3d pt;
  pt = Vector3d(0, 0, 8); m_control_point_vec.push_back(pt);
  pt = Vector3d(2, 1, 7); m_control_point_vec.push_back(pt);
  pt = Vector3d(3, 2, 6); m_control_point_vec.push_back(pt);
  pt = Vector3d(2, 4, 4); m_control_point_vec.push_back(pt);
  pt = Vector3d(1, 5, 3); m_control_point_vec.push_back(pt);
  pt = Vector3d(0, 7, 1); m_control_point_vec.push_back(pt);
  pt = Vector3d(-1, 8, 1); m_control_point_vec.push_back(pt);
}

void IterativePlanner::laneMarkerCallback(const std_msgs::Empty msg)
{
  if (m_gazebo_mode)
    m_target_ptr->laneMarkerVisualization();
  // if (m_route_id == 3 || m_route_id == 4)
  //   m_target_ptr->crossLaneMarkerVisualization();
}


void IterativePlanner::vehicleCurrentPosVisualization(int vehicle_type)
{
  // x,y,z,r,p,y
  // todo: currently directly assign ang value to orientation.w
  if (vehicle_type == 0){
    m_target_ptr->WriteVehicleOctree(0, Pose6D(m_target_odom.pose.pose.position.x+0.8, m_target_odom.pose.pose.position.y, 0.0f, 0.0, 0.0, m_target_odom.pose.pose.orientation.w));
  }
  // truck withour roof
  else if (vehicle_type == -1){
    m_target_ptr->WriteVehicleOctree(-1, Pose6D(m_target_odom.pose.pose.position.x+0.8, m_target_odom.pose.pose.position.y, 0.0f, 0.0, 0.0, m_target_odom.pose.pose.orientation.w));
  }
}

void IterativePlanner::targetTrajParamCallback(const quadrotor_trajectory::TrackParamStampedConstPtr& msg)
{
  /* When in force landing state, do not update target trajectory. */
  if (m_uav.m_uav_state == 8 || m_uav.m_uav_state == 9) // if its in during-force-land or finish-land states, do not need target trajectory any longer
    return;

  double cur_time = msg->header.stamp.toSec();

  /* Publish estimated target position and velocity */
  int traj_order_1 = msg->params.layout.dim[0].size;
  std::vector<double> data_1;
  VehicleTrajectoryBase target_traj_base_1;
  for (int i = 0; i < 2*traj_order_1+1; ++i)
    data_1.push_back(msg->params.data[i]);
  target_traj_base_1.onInit(traj_order_1, data_1);
  Vector3d target_pos_1 = target_traj_base_1.nOrderVehicleTrajectory(0, 0.0);
  Vector3d target_vel_1 = target_traj_base_1.nOrderVehicleTrajectory(1, 0.0);
  nav_msgs::Odometry target_odom_estimated;
  target_odom_estimated.pose.pose.position.x = target_pos_1[0];
  target_odom_estimated.pose.pose.position.y = target_pos_1[1];
  target_odom_estimated.pose.pose.position.z = target_pos_1[2];
  target_odom_estimated.twist.twist.linear.x = target_vel_1[0];
  target_odom_estimated.twist.twist.linear.y = target_vel_1[1];
  target_odom_estimated.twist.twist.linear.z = target_vel_1[2];
  m_pub_estimated_target_odom.publish(target_odom_estimated);

  /* Re-plan */
  /* Conditions: 1, last plan already last more than threshold time; 2, method 2: need to get last trajectory to follow; 3, estimated trajectory is unreliable with time incresing  */
  bool is_estimated_traj_unreliable;
  /* If m_target_traj_base is not assigned value yet */
  if(!m_target_traj_base.m_init_flag)
    is_estimated_traj_unreliable = true;
  else
    is_estimated_traj_unreliable = m_target_traj_base.isVehicleDeviateTrajectory(m_target_traj_deviation_threshold, m_target_odom.pose.pose.position, cur_time - m_vehicle_traj_recv_time);

  if (m_uav.m_uav_state == 7 || cur_time - m_vehicle_traj_recv_time > m_global_planning_period_time || is_estimated_traj_unreliable){
    m_global_planning_period_time = m_global_planning_period_default_time;
    m_vehicle_traj_recv_time = cur_time;

    int traj_order = msg->params.layout.dim[0].size;
    std::vector<double> data;
    for (int i = 0; i < 2*traj_order+1; ++i)
      data.push_back(msg->params.data[i]);
    m_target_traj_base.onInit(traj_order, data);
    if (m_target_traj_param_print_flag)
      m_target_traj_base.printAll();

    /* run Iterative Searching */
    if (m_uav.m_uav_state >= 3 && m_uav.m_uav_state <= 7){ // for states which needs planned trajectory
      runIterativeSearching();

      /* assign param to uav */
      m_uav.m_bspline_traj_ptr = &m_bspline_generator;
      m_uav.m_traj_updated = true;
      m_uav.m_traj_first_updated = true;
      m_uav.m_traj_updated_time = cur_time;
      m_uav.m_target_traj = m_target_traj_base;
    }
  }
}

void IterativePlanner::controlPolygonDisplay(int mode){
  int control_points_num = m_control_point_vec.size();
  //std::cout << "[Display] Control points number: " << control_points_num << "\n";
  int id_cnt = 0;
  visualization_msgs::MarkerArray path_markers;
  visualization_msgs::Marker control_point_marker, line_list_marker;
  // line_array for display target ground truth future data, usually noted.
  visualization_msgs::Marker line_array_target_gt_marker;
  control_point_marker.ns = line_list_marker.ns = "control_polygon";
  control_point_marker.header.frame_id = line_list_marker.header.frame_id = std::string("/world");
  control_point_marker.header.stamp = line_list_marker.header.stamp = ros::Time().now();
  if (mode == 1)
    control_point_marker.action = line_list_marker.action = visualization_msgs::Marker::ADD;
  else
    control_point_marker.action = line_list_marker.action = visualization_msgs::Marker::DELETE;

  line_array_target_gt_marker.header = line_list_marker.header;
  line_array_target_gt_marker.action = line_list_marker.action;
  line_array_target_gt_marker.ns = line_list_marker.ns;

  control_point_marker.type = visualization_msgs::Marker::SPHERE;
  line_list_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_array_target_gt_marker.type = visualization_msgs::Marker::LINE_STRIP;

  line_list_marker.id = id_cnt;
  ++id_cnt;
  line_list_marker.scale.x = 0.1;
  line_list_marker.color.r = 0.0;
  line_list_marker.color.g = 1.0;
  line_list_marker.color.b = 0.0;
  line_list_marker.color.a = 1.0;
  geometry_msgs::Point pt;
  vector3dConvertToPoint(m_control_point_vec[0], pt);
  line_list_marker.points.push_back(pt);
  vector3dConvertToPoint(m_control_point_vec[1], pt);
  line_list_marker.points.push_back(pt);
  for (int i = 2; i < control_points_num; ++i){
    vector3dConvertToPoint(m_control_point_vec[i-2], pt);
    line_list_marker.points.push_back(pt);
    vector3dConvertToPoint(m_control_point_vec[i], pt);
    line_list_marker.points.push_back(pt);
    vector3dConvertToPoint(m_control_point_vec[i-1], pt);
    line_list_marker.points.push_back(pt);
    vector3dConvertToPoint(m_control_point_vec[i], pt);
    line_list_marker.points.push_back(pt);
  }
  path_markers.markers.push_back(line_list_marker);

  for (int i = 0; i < control_points_num; ++i){
    control_point_marker.id = id_cnt;
    ++id_cnt;
    control_point_marker.pose.position.x = m_control_point_vec[i].x();
    control_point_marker.pose.position.y = m_control_point_vec[i].y();
    control_point_marker.pose.position.z = m_control_point_vec[i].z();
    control_point_marker.pose.orientation.x = 0.0;
    control_point_marker.pose.orientation.y = 0.0;
    control_point_marker.pose.orientation.z = 0.0;
    control_point_marker.pose.orientation.w = 1.0;
    if (i == 0 || i == control_points_num-1){
      control_point_marker.scale.x = 1.0;
      control_point_marker.scale.y = 1.0;
      control_point_marker.scale.z = 1.0;
      control_point_marker.color.a = 1;
      control_point_marker.color.r = 0.0f;
      control_point_marker.color.g = 1.0f;
      control_point_marker.color.b = 0.0f;
      path_markers.markers.push_back(control_point_marker);
      continue;
    }
    else{
      control_point_marker.scale.x = 0.5;
      control_point_marker.scale.y = 0.5;
      control_point_marker.scale.z = 0.5;
      control_point_marker.color.a = 1;
      control_point_marker.color.r = 0.0f;
      control_point_marker.color.g = 1.0f;
      control_point_marker.color.b = 0.0f;
      path_markers.markers.push_back(control_point_marker);
    }
  }

  /* Add elements in line strip */
  // line_array_target_gt_marker.id = id_cnt;
  // ++id_cnt;
  // line_array_target_gt_marker.scale.x = 1.0;
  // line_array_target_gt_marker.color.r = 1.0;
  // line_array_target_gt_marker.color.g = 0.0;
  // line_array_target_gt_marker.color.b = 0.0;
  // line_array_target_gt_marker.color.a = 1.0;
  // double sample_gap = 0.1;
  // for (int i = 0; i < (int)(m_landing_time/sample_gap); ++i){
  //   geometry_msgs::Point pt;
  //   vector3dConvertToPoint(nOrderVehicleTrajectoryGroundTruth(0, sample_gap*i), pt);
  //   line_array_target_gt_marker.points.push_back(pt);
  // }
  // path_markers.markers.push_back(line_array_target_gt_marker);

  m_pub_reconstructed_path_markers.publish(path_markers);
}

bool IterativePlanner::isInsideBoarder(Vector3d query_point)
{
  if (query_point.x() >= m_octomap_boarder_val || query_point.x() <= -m_octomap_boarder_val
      || query_point.y() >= m_octomap_boarder_val || query_point.y() <= -m_octomap_boarder_val
      ||query_point.z() >= m_octomap_boarder_val || query_point.z() <= -m_octomap_boarder_val)
    return false;
  else
    return true;
}

bool IterativePlanner::isInsideBoarder(point3d query_point)
{
  if (query_point.x() >= m_octomap_boarder_val || query_point.x() <= -m_octomap_boarder_val
      || query_point.y() >= m_octomap_boarder_val || query_point.y() <= -m_octomap_boarder_val
      ||query_point.z() >= m_octomap_boarder_val || query_point.z() <= -m_octomap_boarder_val)
    return false;
  else
    return true;
}

bool IterativePlanner::getGridCenter(VehicleOctomapServer* obstacle_ptr, Vector3d query_point, Vector3d& center_point, int depth)
{
  bool isGridFree = true;

  if (!isInsideBoarder(query_point)){
    isGridFree = false;
    return false;
  }

  // when not have prior knowledge of depth, assign depth as -1
  if (depth == -1)
    {
      point3d query_point_p(query_point[0], query_point[1], query_point[1]);
      OcTreeNode* result = obstacle_ptr->m_octree->searchReturnDepth(query_point_p, 0, depth);
      if (result != NULL)
        isGridFree = false;
    }
  key_type key_x = obstacle_ptr->m_octree->coordToKey(query_point.x(), depth);
  key_type key_y = obstacle_ptr->m_octree->coordToKey(query_point.y(), depth);
  key_type key_z = obstacle_ptr->m_octree->coordToKey(query_point.z(), depth);
  double center_x = obstacle_ptr->m_octree->keyToCoord(key_x, depth);
  double center_y = obstacle_ptr->m_octree->keyToCoord(key_y, depth);
  double center_z = obstacle_ptr->m_octree->keyToCoord(key_z, depth);
  center_point.x() = center_x;
  center_point.y() = center_y;
  center_point.z() = center_z;
  return isGridFree;
}


bool IterativePlanner::getGridCenter(Vector3d query_point, Vector3d& center_point, int depth)
{
  bool isGridFree = true;

  if (!isInsideBoarder(query_point)){
    isGridFree = false;
    return false;
  }

  // when not have prior knowledge of depth, assign depth as -1
  if (depth == -1)
    {
      point3d query_point_p(query_point[0], query_point[1], query_point[1]);
      std::cout << "query point: " << query_point_p.x() << ", " << query_point_p.y() << ", " << query_point_p.z() << "\n";
      OcTreeNode* result = m_target_ptr->m_octree->searchReturnDepth(query_point_p, 0, depth);
      if (result != NULL)
        isGridFree = false;
    }
  key_type key_x = m_target_ptr->m_octree->coordToKey(query_point.x(), depth);
  key_type key_y = m_target_ptr->m_octree->coordToKey(query_point.y(), depth);
  key_type key_z = m_target_ptr->m_octree->coordToKey(query_point.z(), depth);
  double center_x = m_target_ptr->m_octree->keyToCoord(key_x, depth);
  double center_y = m_target_ptr->m_octree->keyToCoord(key_y, depth);
  double center_z = m_target_ptr->m_octree->keyToCoord(key_z, depth);
  center_point.x() = center_x;
  center_point.y() = center_y;
  center_point.z() = center_z;
  return isGridFree;
}

bool IterativePlanner::getGridCenter(point3d query_point, point3d& center_point, int depth)
{
  bool isGridFree = true;

  if (!isInsideBoarder(query_point)){
    isGridFree = false;
    return false;
  }

  // when not have prior knowledge of depth, assign depth as -1
  if (depth == -1)
    {
      OcTreeNode* result = m_target_ptr->m_octree->searchReturnDepth(query_point, 0, depth);
      if (result != NULL)
        isGridFree = false;
    }

  key_type key_x = m_target_ptr->m_octree->coordToKey(query_point.x(), depth);
  key_type key_y = m_target_ptr->m_octree->coordToKey(query_point.y(), depth);
  key_type key_z = m_target_ptr->m_octree->coordToKey(query_point.z(), depth);
  double center_x = m_target_ptr->m_octree->keyToCoord(key_x, depth);
  double center_y = m_target_ptr->m_octree->keyToCoord(key_y, depth);
  double center_z = m_target_ptr->m_octree->keyToCoord(key_z, depth);
  center_point.x() = center_x;
  center_point.y() = center_y;
  center_point.z() = center_z;
  return isGridFree;
}

inline void IterativePlanner::vector3dConvertToPoint32(Vector3d point3, geometry_msgs::Point32& point32)
{
  point32.x = point3.x();
  point32.y = point3.y();
  point32.z = point3.z();
}

inline void IterativePlanner::vector3dConvertToPoint(Vector3d point3, geometry_msgs::Point& point)
{
  point.x = point3.x();
  point.y = point3.y();
  point.z = point3.z();
}

void IterativePlanner::targetOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  /* uav */
  m_uav.getTargetOdom(msg);

  m_target_odom = *msg;

  if (m_gazebo_mode){
    double cur_time = ros::Time().now().toSec();
    if (cur_time - m_vehicles_visualize_prev_time > m_vehicles_visualize_period_time){
      m_vehicles_visualize_prev_time = cur_time;
      vehicleCurrentPosVisualization(0);
      m_target_ptr->publishVehicleAll(ros::Time().now());
      m_target_ptr->m_octree->clear();
    }
  }
}

void IterativePlanner::uavStartFlagCallback(const std_msgs::Empty msg)
{
  m_uav.m_uav_state = 1;
}

void IterativePlanner::uavStraightLaneLandingStartFlagCallback(const std_msgs::Empty msg)
{
  m_uav.m_uav_state = 5; // state: start to land
  ROS_INFO("[Change to state] Start to land!");
}

void IterativePlanner::uavArriveGpsPointFlagCallback(const std_msgs::Empty msg)
{
  m_uav.m_uav_arrive_gps_point_flag = true;
  ROS_INFO("[Change to state] State enable. Arrive to gps pt!");
}

void IterativePlanner::uavOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  m_uav_odom = *msg;
  /* state: 0, still; 1, taking off; 2, ready to move; 3, start to track; 4, wait to land; 5, start to land; 6, wait to force land; 7, start force land; 8, during force land; 9, finish force land */
  m_uav.getUavOdom(msg);
  if (m_uav.m_uav_state == 1){
    m_uav.uavMovingToPresetHeight(10.0);
    m_pub_uav_cmd.publish(m_uav.m_uav_cmd);
  }
  else if (m_uav.m_uav_state == 2){
    if (m_uav.uavTargetHorizonDistance() < 6.0){
      m_uav.m_uav_state = 3;
    }
  }
  else if (m_uav.m_uav_state == 8){ // state: during land
    if (m_uav_force_landing_method == 2) // method 2, follow fixed planned trajectory
      m_uav.trackGlobalTrajectory();
    else if (m_uav_force_landing_method == 1){
      if (m_uav_force_landing_start_time < 0){ // not assigned value yet
        m_uav_force_landing_start_time = msg->header.stamp.toSec();
      }
      else if (msg->header.stamp.toSec() - m_uav_force_landing_start_time > 0.6){ // 0.6s: roughly estimated from force landing height and force landing velocity
        m_uav.m_uav_state = 9; // change to finish land state
        ROS_INFO("[Change to state] Finish landing.");
      }
    }
    m_pub_uav_cmd.publish(m_uav.m_uav_cmd);
  }
  else if (m_uav.m_uav_state == 9){ // state: finish land
    m_uav.m_uav_cmd.linear.x = 0.0;
    m_uav.m_uav_cmd.linear.y = 0.0;
    m_uav.m_uav_cmd.linear.z = -0.5;
    m_pub_uav_cmd.publish(m_uav.m_uav_cmd);
  }
  else if (m_uav.m_uav_state >= 3){ // track the planned trajectory
    /* In case traj not being calculated before state changes to 3 */
    /* uav_arrive_gps_point_flag comes, then state changes could be enabled */
    if (m_uav.m_traj_first_updated && m_uav.m_uav_arrive_gps_point_flag){ // in case odom topic comes before first trajectory topic
      if (m_relative_traj_track_mode)
	m_uav.trackTrajectory();
      else
	m_uav.trackGlobalTrajectory();
      m_pub_uav_cmd.publish(m_uav.m_uav_cmd);
    }
  }
}

void IterativePlanner::restrictedRegionCenterCallback(const nav_msgs::OdometryConstPtr& msg)
{
  m_restricted_region_recv_flag = true;
  m_restricted_region_center_pos[0] = msg->pose.pose.position.x;
  m_restricted_region_center_pos[1] = msg->pose.pose.position.y;
  m_restricted_region_center_pos[2] = msg->pose.pose.position.z;
}

void IterativePlanner::restrictedControlPoint(Vector3d& pt)
{
  double disance_to_center = sqrt(pow(pt[0]-m_restricted_region_center_pos[0], 2.0) + pow(pt[1]-m_restricted_region_center_pos[1], 2.0));
  if (disance_to_center > m_restricted_region_radius){
    pt[0] = (pt[0]-m_restricted_region_center_pos[0]) * m_restricted_region_radius / disance_to_center + m_restricted_region_center_pos[0];
    pt[1] = (pt[1]-m_restricted_region_center_pos[1]) * m_restricted_region_radius / disance_to_center + m_restricted_region_center_pos[1];
  }
}
