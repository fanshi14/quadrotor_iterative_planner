
#include <truck_server/QuadrotorCommand.h>

QuadrotorCommand::QuadrotorCommand() {}
void QuadrotorCommand::onInit()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("debug_mode", m_debug_mode, true);
  private_nh.param("verbose", m_verbose, false);
  private_nh.param("dji_mode", m_dji_mode, true);
  private_nh.param("landing_mode", m_landing_mode, true);
  private_nh.param("global_coordinate_control", m_global_coordinate_control_mode, false);
  private_nh.param("uav_vel_upper_bound", m_uav_vel_ub, 7.0);
  private_nh.param("uav_vel_lower_bound", m_uav_vel_lb, -7.0);
  private_nh.param("uav_acc_upper_bound", m_uav_acc_ub, 2.0);
  private_nh.param("uav_acc_lower_bound", m_uav_acc_lb, -2.0);
  private_nh.param("uav_cmd_traj_track_p_gain", m_traj_track_p_gain, 0.3);
  private_nh.param("uav_cmd_traj_track_i_gain", m_traj_track_i_gain, 0.03);
  private_nh.param("uav_cmd_traj_track_d_gain", m_traj_track_d_gain, 0.0);
  private_nh.param("uav_cmd_traj_track_p_term_max", m_traj_track_p_term_max, 6.0);
  private_nh.param("uav_cmd_traj_track_i_term_max", m_traj_track_i_term_max, 4.0);
  private_nh.param("uav_cmd_traj_track_d_term_max", m_traj_track_d_term_max, 0.0);
  private_nh.param("uav_odom_freq", m_uav_odom_freq, 50.0);
  private_nh.param("target_height", m_target_height, 1.8);
  private_nh.param("uav_tracking_landing_constant_speed", m_uav_tracking_landing_constant_speed, -0.4);
  private_nh.param("uav_landing_constant_speed", m_uav_landing_constant_speed, -1.0);
  private_nh.param("uav_start_landing_height_upperbound", m_uav_start_landing_height_upperbound, 5.0);
  private_nh.param("uav_force_landing_height_upperbound", m_uav_force_landing_height_upperbound, 0.4);
  private_nh.param("uav_start_landing_cnt_thresh", m_uav_start_landing_cnt_thresh, 0);
  private_nh.param("uav_force_landing_cnt_thresh", m_uav_force_landing_cnt_thresh, 5);
  private_nh.param("uav_force_landing_method", m_uav_force_landing_method, 1);

  private_nh.param("uav_gliding_mode_pub_topic_name", m_uav_gliding_mode_pub_topic_name, std::string("/uav_start_gliding"));

  m_pub_uav_gliding_mode = m_nh.advertise<std_msgs::Empty>(m_uav_gliding_mode_pub_topic_name, 1);

  m_traj_track_i_term_accumulation.setValue(0.0, 0.0, 0.0);
  m_uav_yaw_i_term_accumulation = 0.0;
  m_traj_updated = false;
  m_traj_first_updated = false;
  m_uav_arrive_gps_point_flag = false;

  m_uav_cmd.linear.x = 0.0; m_uav_cmd.linear.y = 0.0; m_uav_cmd.linear.z = 0.0;
  m_uav_cmd.angular.x = 0.0; m_uav_cmd.angular.y = 0.0; m_uav_cmd.angular.z = 0.0;

  /* state: 0, still; 1, taking off; 2, ready to move; 3, start to track; 4, wait to land; 5, start to land; 6, wait to force land; 7, start force land; 8, during force land; 9, finish force land */
  m_uav_state = 0;

  m_uav_start_landing_cnt = 0;
  m_uav_force_landing_cnt = 0;

  sleep(0.2); //To collect initial values for truck and uav odom, which will be used in following functions


}

QuadrotorCommand::~QuadrotorCommand() {}

void QuadrotorCommand::getTruckOdom(const nav_msgs::OdometryConstPtr& truck_odom_msg)
{
  m_truck_odom = *truck_odom_msg;
  m_truck_world_pos.setValue(truck_odom_msg->pose.pose.position.x,
                             truck_odom_msg->pose.pose.position.y,
                             truck_odom_msg->pose.pose.position.z);
}

void QuadrotorCommand::getUavOdom(const nav_msgs::OdometryConstPtr& uav_odom_msg)
{
  m_uav_odom = *uav_odom_msg;
  m_uav_world_pos.setValue(uav_odom_msg->pose.pose.position.x,
                           uav_odom_msg->pose.pose.position.y,
                           uav_odom_msg->pose.pose.position.z);
  m_uav_q = tf::Quaternion(uav_odom_msg->pose.pose.orientation.x,
                           uav_odom_msg->pose.pose.orientation.y,
                           uav_odom_msg->pose.pose.orientation.z,
                           uav_odom_msg->pose.pose.orientation.w);
  m_uav_world_vel.setValue(uav_odom_msg->twist.twist.linear.x,
                           uav_odom_msg->twist.twist.linear.y,
                           uav_odom_msg->twist.twist.linear.z);
}

void QuadrotorCommand::trackTrajectory()
{
  // Control
  if (m_traj_updated){
    m_traj_updated = false;
    m_bspline_traj_ptr->getDerive();
    // ??
    m_traj_track_i_term_accumulation.setValue(0.0, 0.0, 0.0);
    m_uav_yaw_i_term_accumulation = 0.0;
  }
  double uav_current_traj_time = m_uav_odom.header.stamp.toSec() - m_traj_updated_time;
  if (uav_current_traj_time < m_bspline_traj_ptr->m_t0){
    if(m_verbose)
      {
        ROS_WARN("Current odom time is less than bspline start time. ");
        std::cout << "t0: " << m_bspline_traj_ptr->m_t0 << ", odom time: " << uav_current_traj_time << "\n";
      }
    uav_current_traj_time = m_bspline_traj_ptr->m_t0;
  }
  else if (uav_current_traj_time > m_bspline_traj_ptr->m_tn){
    if(m_verbose)
      {
        ROS_WARN("Current odom time is larger than bspline end time. ");
        std::cout << "tn: " << m_bspline_traj_ptr->m_tn << ", odom time: " << uav_current_traj_time << "\n";
      }
    uav_current_traj_time = m_bspline_traj_ptr->m_tn;
  }
  tf::Vector3 uav_des_world_vel = vectorToVector3(m_bspline_traj_ptr->evaluateDerive(uav_current_traj_time));
  tf::Vector3 uav_des_world_pos = vectorToVector3(m_bspline_traj_ptr->evaluate(uav_current_traj_time));
  tf::Vector3 truck_des_world_pos = vector3dToVector3(m_truck_traj.nOrderVehicleTrajectory(0, uav_current_traj_time) + Vector3d(0.0, 0.0, m_target_height));
  tf::Vector3 uav_des_truck_pos = uav_des_world_pos - truck_des_world_pos;
  tf::Vector3 uav_real_truck_pos;
  uav_real_truck_pos = m_uav_world_pos - m_truck_world_pos; // z axis for truck_world_pos equals to 0.0

  tf::Matrix3x3  uav_rot_mat(m_uav_q);
  tfScalar uav_roll, uav_pitch, uav_yaw;
  uav_rot_mat.getRPY(uav_roll, uav_pitch, uav_yaw);
  tf::Matrix3x3 r_z; r_z.setRPY(0, 0, uav_yaw);

  /* pid control in trajectory tracking */
  tf::Vector3 traj_track_p_term =  (uav_des_truck_pos - uav_real_truck_pos) * m_traj_track_p_gain;
  double p_term_absolute_value = traj_track_p_term.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (p_term_absolute_value > m_traj_track_p_term_max)
    traj_track_p_term = traj_track_p_term * m_traj_track_p_term_max / p_term_absolute_value;
  m_traj_track_i_term_accumulation += (uav_des_truck_pos - uav_real_truck_pos) / m_uav_odom_freq;
  double i_term_accumulation_absolute_value = m_traj_track_i_term_accumulation.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (i_term_accumulation_absolute_value > m_traj_track_i_term_max)
    m_traj_track_i_term_accumulation = m_traj_track_i_term_accumulation * m_traj_track_i_term_max / i_term_accumulation_absolute_value;
  tf::Vector3 traj_track_i_term = m_traj_track_i_term_accumulation * m_traj_track_i_gain;

  /* feedforward */
  tf::Vector3 uav_vel = uav_des_world_vel + traj_track_p_term + traj_track_i_term;
  double uav_vel_absolute_value = uav_vel.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (uav_vel_absolute_value > m_uav_vel_ub)
    uav_vel = uav_vel * m_uav_vel_ub / uav_vel_absolute_value;

  /* Judge if the controller is locally based on uav coordinate. */
  if (!m_global_coordinate_control_mode){
    uav_vel = r_z.inverse() * uav_vel;
  }

  m_uav_cmd.linear.x = uav_vel.getX();
  m_uav_cmd.linear.y = uav_vel.getY();
  double landing_vel_z = 0.0;
  if (m_landing_mode){
    double target_distance_xy = sqrt(pow(uav_real_truck_pos[0], 2) + pow(uav_real_truck_pos[1], 2));
    if (m_uav_state == 3){ // start to track and reach to specific height
      if (uav_real_truck_pos[2] < m_uav_start_landing_height_upperbound + m_target_height){
        m_uav_state = 4;
        std::cout << uav_real_truck_pos[2] << ", " << m_uav_world_pos[2] << ", " << m_uav_start_landing_height_upperbound + m_target_height << "\n\n";
        ROS_INFO("[Change to state] Wait to land!");
      }
      else
        landing_vel_z = m_uav_tracking_landing_constant_speed;
    }
    else if (m_uav_state == 4){ // wait to land in straight lane
      // todo: depends on gps information

    }
    else if (m_uav_state == 5){ //start to land
      if (uav_real_truck_pos[2] < m_uav_force_landing_height_upperbound + m_target_height){
        ROS_INFO("[Change to state] Waiting force landing!!");
        m_uav_state = 6;
        m_uav_force_landing_cnt += 1;
      }
      else{
        if (m_uav_start_landing_cnt > m_uav_start_landing_cnt_thresh)
          landing_vel_z = m_uav_landing_constant_speed;
        else{
          if (target_distance_xy < 2.0)
            m_uav_start_landing_cnt += 1;
        }
      }
    }
    else if (m_uav_state == 6){ // wait for force landing
      /* Force landing requires uav has several frames in the exact center of target */
      if (m_uav_force_landing_cnt > m_uav_force_landing_cnt_thresh){
        /* Using force landing trajectory requires this trajectory is newly estimated, which would guarantee good accuracy in near future. */
        ROS_INFO("[Change to state] Prepare to force landing.");
        m_uav_state = 7; // start force landing
        /* force landing with same speed of truck */
        if (m_uav_force_landing_method == 1){ //method 1, follow fixed truck velocity
          tf::Vector3 truck_des_world_vel = vector3dToVector3(m_truck_traj.nOrderVehicleTrajectory(1, uav_current_traj_time));
          tf::Vector3 truck_des_uav_vel = r_z.inverse() * truck_des_world_vel;
          m_uav_state = 8; // since velocity in force landing is fixed in method 1, we could already view it in state 8: during force land. For method 2, it needs generate last trajectory to get into state 8.
          m_uav_cmd.linear.x = truck_des_uav_vel[0];
          m_uav_cmd.linear.y = truck_des_uav_vel[1];
          m_uav_cmd.linear.z = -1.5;
          return;
        }
        else if (m_uav_force_landing_method == 3){ //method 3, gliding mode
          /* start gliding mode: attitude control kepping body level and stably descending */
          std_msgs::Empty msg;
          m_pub_uav_gliding_mode.publish(msg);
          return;
        }
      }
      else{
        if (target_distance_xy < 0.5)
          m_uav_force_landing_cnt += 1;
      }
    }
  }
  m_uav_cmd.linear.z = landing_vel_z;
}


void QuadrotorCommand::trackGlobalTrajectory() // for method 2
{
  if (m_uav_state == 9){
    m_uav_cmd.linear.x = 0.0;
    m_uav_cmd.linear.y = 0.0;
    m_uav_cmd.linear.z = -1.5;
    return;
  }
  double uav_current_traj_time = m_uav_odom.header.stamp.toSec() - m_traj_updated_time;
  if (uav_current_traj_time < m_bspline_traj_ptr->m_t0){
    if(m_verbose)
      {
        ROS_WARN("Current odom time is less than bspline start time. ");
        std::cout << "t0: " << m_bspline_traj_ptr->m_t0 << ", odom time: " << uav_current_traj_time << "\n";
      }
    uav_current_traj_time = m_bspline_traj_ptr->m_t0;
  }
  else if (uav_current_traj_time > m_bspline_traj_ptr->m_tn){
    if(m_verbose)
      {
        ROS_WARN("Current odom time is larger than bspline end time. ");
        std::cout << "tn: " << m_bspline_traj_ptr->m_tn << ", odom time: " << uav_current_traj_time << "\n";
      }
    uav_current_traj_time = m_bspline_traj_ptr->m_tn;
    ROS_INFO("[Change to state] UAV should already land. ");
    m_uav_cmd.linear.x = 0.0;
    m_uav_cmd.linear.y = 0.0;
    m_uav_cmd.linear.z = -1.5;
    m_uav_state = 9;
    return;
  }
  tf::Vector3 uav_des_world_vel = vectorToVector3(m_bspline_traj_ptr->evaluateDerive(uav_current_traj_time));
  tf::Vector3 uav_des_world_pos = vectorToVector3(m_bspline_traj_ptr->evaluate(uav_current_traj_time));
  tf::Vector3 truck_des_world_pos = vector3dToVector3(m_truck_traj.nOrderVehicleTrajectory(0, uav_current_traj_time) + Vector3d(0.0, 0.0, m_target_height));
  tf::Vector3 uav_real_world_pos;
  uav_real_world_pos = m_uav_world_pos;

  tf::Matrix3x3  uav_rot_mat(m_uav_q);
  tfScalar uav_roll, uav_pitch, uav_yaw;
  uav_rot_mat.getRPY(uav_roll, uav_pitch, uav_yaw);
  tf::Matrix3x3 r_z; r_z.setRPY(0, 0, uav_yaw);

  /* pid control in trajectory tracking */
  tf::Vector3 traj_track_p_term =  (uav_des_world_pos - uav_real_world_pos) * m_traj_track_p_gain;
  double p_term_absolute_value = traj_track_p_term.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (p_term_absolute_value > m_traj_track_p_term_max)
    traj_track_p_term = traj_track_p_term * m_traj_track_p_term_max / p_term_absolute_value;
  m_traj_track_i_term_accumulation += (uav_des_world_pos - uav_real_world_pos) / m_uav_odom_freq;
  double i_term_accumulation_absolute_value = m_traj_track_i_term_accumulation.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (i_term_accumulation_absolute_value > m_traj_track_i_term_max)
    m_traj_track_i_term_accumulation = m_traj_track_i_term_accumulation * m_traj_track_i_term_max / i_term_accumulation_absolute_value;
  tf::Vector3 traj_track_i_term = m_traj_track_i_term_accumulation * m_traj_track_i_gain;

  /* feedforward */
  tf::Vector3 uav_vel = uav_des_world_vel + traj_track_p_term + traj_track_i_term;
  double uav_vel_absolute_value = uav_vel.distance(tf::Vector3(0.0, 0.0, 0.0));
  if (uav_vel_absolute_value > m_uav_vel_ub)
    uav_vel = uav_vel * m_uav_vel_ub / uav_vel_absolute_value;

  /* Judge if the controller is locally based on uav coordinate. */
  if (!m_global_coordinate_control_mode){
    uav_vel = r_z.inverse() * uav_vel;
  }

  m_uav_cmd.linear.x = uav_vel.getX();
  m_uav_cmd.linear.y = uav_vel.getY();
  m_uav_cmd.linear.z = uav_vel.getZ();
}

bool QuadrotorCommand::uavMovingToPresetHeight(double height)
{
  m_uav_state = 1;
  m_uav_cmd.linear.x = 0.0; m_uav_cmd.linear.y = 0.0; m_uav_cmd.linear.z = 0.0;
  if (m_uav_world_pos.getZ() < height){
    m_uav_cmd.linear.z = 1.0;
  }
  else if (m_uav_world_pos.getZ() > height + 1.0){
    m_uav_cmd.linear.z = -1.0;
  }
  else{
    m_uav_state = 2;
    ROS_INFO("[Change to state] UAV reached specific height.");
    return true;
  }
  return false;
}

double QuadrotorCommand::uavTruckHorizonDistance()
{
  return pow((m_uav_world_pos.getX()-m_truck_world_pos.getX()), 2) + pow((m_uav_world_pos.getY()-m_truck_world_pos.getY()), 2);
}

inline tf::Vector3 QuadrotorCommand::vectorToVector3(std::vector<double> vec)
{
  tf::Vector3 vec3(vec[0], vec[1], vec[2]);
  return vec3;
}

inline tf::Vector3 QuadrotorCommand::vector3dToVector3(Vector3d vec)
{
  tf::Vector3 vec3(vec[0], vec[1], vec[2]);
  return vec3;
}
