#include <mkmr_scripts/ManualControllers.h>

ManualControllers::ManualControllers(ros::NodeHandle &nh, ros::NodeHandle &priv_nh):
    nh_(nh),
    priv_nh_ (priv_nh),
    robot_id_{priv_nh.param<std::string>("robot_id", "UNKNOWN")}
{
    std::string lidar_topic;
    std::string runstop_priority_topic;
    std::string runstop_vel_topic;
    std::string slow_speed_info_topic;
    std::string normal_speed_info_topic;


    priv_nh.param<float>("safety_offset", safety_offset_, 0.05f);
    priv_nh.param<float>("ignore_dist", ignore_dist_, 2.0f);
    priv_nh.param<float>("statistical_err", statistical_err_, 0.02f);
    priv_nh.param<float>("systematic_err", systematic_err_, 0.06f);

    priv_nh.param<float>("lidar_x", lidar_x_, 0.07f);
    priv_nh.param<float>("lidar_min_range", lidar_min_range_, 0.05f);

    priv_nh.param<double>("slow_speed", slow_speed_, 0.2);
    priv_nh.param<double>("normal_speed", normal_speed_, 0.4);
    priv_nh.param<double>("fast_speed", fast_speed_, 0.6);

    priv_nh.param<float>("slow_dist", slow_dist_, 0);
    priv_nh.param<float>("slow_side_dist", slow_side_dist_, 0);
    priv_nh.param<float>("normal_dist", normal_dist_, 0);
    priv_nh.param<float>("normal_side_dist", normal_side_dist_, 0);

    priv_nh.param<float>("robot_length_half_x", robot_length_half_x_, 0.7f); // vertical
    priv_nh.param<float>("robot_length_half_y", robot_length_half_y_, 0.3f); // horizontal

    max_uncertainity_ = powf(statistical_err_, 2) + powf(systematic_err_,2);
    max_uncertainity_ = sqrtf(max_uncertainity_);

    /* cosmos syntax 
    \le --> <=
    \left\ --> {
    \right\ --> }
    _{xx} --> subscript xx
    */

    /* safety zone (red)
    -r_{hor}-l_{safetyoffset}\le y\le r_{hor}+l_{safetyoffset}\left\{-x_{lidar}-r_{ver}-l_{safetyoffset}\le x\le l_{safetyoffset}+r_{ver}-x_{lidar}\right\}
    */

    ROS_INFO_STREAM( "********************************");
    ROS_INFO_STREAM( "Stop Zone Min x: " << -lidar_x_ -robot_length_half_x_ - safety_offset_);
    ROS_INFO_STREAM( "Stop Zone Max x: " << safety_offset_ + robot_length_half_x_ - lidar_x_ );
    ROS_INFO_STREAM( "Stop Zone Min y: " << -robot_length_half_y_ - safety_offset_);
    ROS_INFO_STREAM( "Stop Zone Max y: " << robot_length_half_y_  + safety_offset_);

    slow_min_x_ = -slow_dist_ - lidar_x_;
    slow_max_x_ =  slow_dist_ - lidar_x_;
    float abs_side = slow_side_dist_ + robot_length_half_y_ ;
    slow_min_y_ = -1.0f * abs_side;
    slow_max_y_ =  abs_side;

    /* warning zone (yellow)
    -r_{slowdown}-x_{lidar}<x<r_{slowdown}-x_{lidar}\ \left\{-l_{sidedist}-r_{hor}\le y\le l_{sidedist}+r_{hor}\right\}
    */

    ROS_INFO_STREAM( "********************************");
    ROS_INFO_STREAM( "Slow Speed Zone Min x: " << slow_min_x_);
    ROS_INFO_STREAM( "Slow Speed Zone Max x: " << slow_max_x_);
    ROS_INFO_STREAM( "Slow Speed Zone Min y: " << slow_min_y_);
    ROS_INFO_STREAM( "Slow Speed Zone Max y: " << slow_max_y_);

    normal_speed_min_x_ = -normal_dist_ - lidar_x_;
    normal_speed_max_x_ =  normal_dist_ - lidar_x_;
    abs_side = normal_side_dist_ + robot_length_half_x_ ;
    normal_speed_min_y_ = -1.0f * abs_side;
    normal_speed_max_y_ =  abs_side;
   
    /* warning zone (blue)
    -r_{normal}-x_{lidar}<x<r_{normal}-x_{lidar}\ \left\{-l_{normalsidedist}-r_{ver}\le y\le l_{normalsidedist}+r_{ver}\right\}
    */

    ROS_INFO_STREAM( "********************************");
    ROS_INFO_STREAM( "Normal Speed Zone Min x: " << normal_speed_min_x_);
    ROS_INFO_STREAM( "Normal Speed Zone Max x: " << normal_speed_max_x_);
    ROS_INFO_STREAM( "Normal Speed Zone Min y: " << normal_speed_min_y_);
    ROS_INFO_STREAM( "Normal Speed Zone Max y: " << normal_speed_max_y_);



    ROS_INFO_STREAM( "********************************");
    ROS_INFO_STREAM( "Lidar Systematic Error: " << systematic_err_);
    ROS_INFO_STREAM( "Lidar Statistical Error: " << statistical_err_);
    ROS_INFO_STREAM( "Maximum uncertainty: " << max_uncertainity_);
    ROS_INFO_STREAM( "Min reading to stop: " << max_uncertainity_ + safety_offset_);


    priv_nh.param<std::string>("lidar_topic", lidar_topic, "scan");
    priv_nh.param<std::string>("runstop_priority_topic", runstop_priority_topic,  "soft_runstop_priority");
    priv_nh.param<std::string>("runstop_vel_topic", runstop_vel_topic,  "soft_runstop_cmd_vel");
    

    planner_setparam_srv_name_ =  "move_base/TebLocalPlannerROS/set_parameters";
    slow_speed_info_topic =  "slow_speed_enabled";
    normal_speed_info_topic = "normal_speed_enabled";

    soft_runstop_priority_pub_ = nh_.advertise<std_msgs::Bool>(runstop_priority_topic, 2, true);
    soft_runstop_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(runstop_vel_topic, 2, true);
    slow_speed_info_pub_ = nh_.advertise<std_msgs::Bool>(slow_speed_info_topic, 2, true);
    normal_speed_info_pub_ = nh_.advertise<std_msgs::Bool>(normal_speed_info_topic, 2, true);
  

    soft_runstop_vel_pub_.publish(zero_vel_msg_);
    soft_runstop_priority_pub_.publish(disable_priority_msg_);
    slow_speed_info_pub_.publish(disable_priority_msg_);

    enable_priority_msg_.data = true;
    disable_priority_msg_.data = false; 

    speed_ctrl_timer_ = ros::Timer( priv_nh_.createTimer(
        ros::Duration(1 / 20.0), &ManualControllers::speedControllerTimerCb, this));
    runstop_ctrl_timer_ = ros::Timer( priv_nh_.createTimer(
        ros::Duration(1 / 20.0), &ManualControllers::runstopControllerTimerCb, this));


    max_vel_x_param_.name = "max_vel_x";
    max_vel_x_param_.value = normal_speed_;
    dynamic_reconfigure::Config dynrec_conf;
    dynrec_conf.doubles.push_back(max_vel_x_param_);
    srv_req_.config = dynrec_conf;

    sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(lidar_topic, nh_);
    if (msg)
    {
        lidar_min_rad_ = msg->angle_min;
        lidar_max_rad_ = msg->angle_max;
        lidar_incr_rad_ = msg->angle_increment;

        lidar_msg_range_size_ = msg->ranges.size();
        float cur_angle_rad = lidar_min_rad_;
        for (size_t i = 0;i < lidar_msg_range_size_; i++)
        {
            lidar_msg_angles_.push_back(cur_angle_rad);
            cur_angle_rad += lidar_incr_rad_;
        }
    }
    lidar_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(lidar_topic, 1, &ManualControllers::lidarCb, this);
}

ManualControllers::~ManualControllers()
{

}


void ManualControllers::lidarCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    cur_runstop_ = false;
    cur_speed_mode_ = SpeedControl::FAST;
    for (size_t i = 0;i < lidar_msg_range_size_; i++)
    {
        if (msg->ranges[i] < lidar_min_range_ || std::isinf(msg->ranges[i]))
        {
            continue; 
        }
        if (isTheObstacleDangerous(msg->ranges[i], lidar_msg_angles_[i]))
        {
            cur_runstop_ = true;
            cur_speed_mode_ = SpeedControl::SLOW;
            break;
        }
    }
}


inline bool ManualControllers::isCurrentPointDangerous()
{

    if (shouldStop())
    {
        return true;
    }

    if (shouldSlow())
    {
        cur_speed_mode_ = SpeedControl::SLOW;
    }

    if (shouldTravelAtNormalSpeed() && cur_speed_mode_ != SpeedControl::SLOW)
    {
        cur_speed_mode_ = SpeedControl::NORMAL;
    }

    return false;
}


inline bool ManualControllers::shouldStop()
{ 
    return (cur_point_x_ >= -lidar_x_ -robot_length_half_x_ - safety_offset_ &&
            cur_point_x_ <= safety_offset_ + robot_length_half_x_ - lidar_x_  &&
            cur_point_y_ >= -robot_length_half_y_ - safety_offset_ &&
            cur_point_y_ <= robot_length_half_y_  + safety_offset_);  

}

inline bool ManualControllers::shouldSlow()
{
    return (cur_point_x_ > slow_min_x_ &&
            cur_point_x_ < slow_max_x_ &&
            cur_point_y_ > slow_min_y_ &&
            cur_point_y_ < slow_max_y_);
}

inline bool ManualControllers::shouldTravelAtNormalSpeed()
{
    return (cur_point_x_ > normal_speed_min_x_ &&
            cur_point_x_ < normal_speed_max_x_ &&
            cur_point_y_ > normal_speed_min_y_ &&
            cur_point_y_ < normal_speed_max_y_);
}

inline bool ManualControllers::isTheObstacleDangerous(const float &range, const float &angle)
{
    if (range > ignore_dist_)
    {
        return false;
    }

    cur_point_x_ = (range-max_uncertainity_) * cos(angle);
    cur_point_y_ = (range-max_uncertainity_) * sin(angle);

    return isCurrentPointDangerous();

    return false;
}

inline void ManualControllers::enableRunstop()
{
    soft_runstop_priority_pub_.publish(enable_priority_msg_);
    soft_runstop_vel_pub_.publish(zero_vel_msg_);
    ROS_WARN("Soft Runstop enabled");
}

inline void ManualControllers::disableRunstop()
{
    soft_runstop_priority_pub_.publish(disable_priority_msg_);
    ROS_INFO_THROTTLE(2.0, "Soft Runstop disabled");
}

void ManualControllers::runstopControllerTimerCb(const ros::TimerEvent &event)
{
    if (!prev_runstop_ && cur_runstop_)
    {
        enableRunstop();
    }
    if (prev_runstop_ && !cur_runstop_)
    {
        disableRunstop();
    }
    prev_runstop_ = cur_runstop_;
}

void ManualControllers::speedControllerTimerCb(const ros::TimerEvent &event)
{
    if (prev_speed_mode_ != cur_speed_mode_)
    {
        switch (cur_speed_mode_)
        {
            case SpeedControl::SLOW:
                ROS_INFO_STREAM("SLOW");
                slow_speed_info_pub_.publish(enable_priority_msg_);
                setMaxPlannerSpeed(slow_speed_);
                break;
            case SpeedControl::NORMAL:
                ROS_INFO_STREAM("NORMAL");
                normal_speed_info_pub_.publish(enable_priority_msg_);
                setMaxPlannerSpeed(normal_speed_);
                break;
            case SpeedControl::FAST:
                ROS_INFO_STREAM("FAST");
                normal_speed_info_pub_.publish(disable_priority_msg_);
                setMaxPlannerSpeed(fast_speed_);
                break;
            ROS_ERROR_STREAM("Wrong SpeedControl value passed to the cur_speed_mode: " << cur_speed_mode_);
        }
    }
    prev_speed_mode_ = cur_speed_mode_;
}

inline void ManualControllers::setMaxPlannerSpeed(const double &speed)
{
    max_vel_x_param_.value = speed;
    dynamic_reconfigure::Config dynrec_conf;
    dynrec_conf.doubles.push_back(max_vel_x_param_);
    srv_req_.config = dynrec_conf;
    ROS_INFO_STREAM("Setting move_base speed: " << speed);
    ros::service::call(planner_setparam_srv_name_, srv_req_, srv_resp_);
}
