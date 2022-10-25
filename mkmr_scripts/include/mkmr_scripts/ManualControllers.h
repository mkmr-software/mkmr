#ifndef MANUALCONTROLLERS_H
#define MANUALCONTROLLERS_H

#include <ros/ros.h>
#include <ros/master.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>


#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


enum SpeedControl
{
    SLOW= 0,
    NORMAL = 1,
    FAST = 2
};

class ManualControllers
{
public:

    ManualControllers(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
    ~ManualControllers();

private:
    inline bool isEqualEnough(const float &a, const float &b);
    inline void enableRunstop();
    inline void disableRunstop();
    inline void setMaxPlannerSpeed(const double &speed);
    inline bool isTheObstacleDangerous(const float &range, const float &angle);
    inline bool isCurrentPointDangerous();
    inline bool shouldSlow();
    inline bool shouldTravelAtNormalSpeed();
    inline bool shouldStop();

    void lidarCb(const sensor_msgs::LaserScan::ConstPtr &msg);

    void speedControllerTimerCb(const ros::TimerEvent & event);
    void runstopControllerTimerCb(const ros::TimerEvent & event);

    bool cur_runstop_{false};
    bool prev_runstop_{false};

    bool in_danger_circle_{false};
    bool in_danger_square_{false};

    bool enable_safe_control_{false};


    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher soft_runstop_priority_pub_;
    ros::Publisher soft_runstop_vel_pub_;
    ros::Publisher slow_speed_info_pub_;
    ros::Publisher normal_speed_info_pub_;

    ros::Timer speed_ctrl_timer_;
    ros::Timer runstop_ctrl_timer_;

    const geometry_msgs::Twist zero_vel_msg_;
    std_msgs::Bool disable_priority_msg_;
    std_msgs::Bool enable_priority_msg_;

    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_resp_;
    dynamic_reconfigure::DoubleParameter max_vel_x_param_;


    float ignore_dist_;
    float statistical_err_;
    float systematic_err_;
    float max_uncertainity_;

    SpeedControl cur_speed_mode_{SpeedControl::SLOW};
    SpeedControl prev_speed_mode_{SpeedControl::SLOW};

    float slow_dist_;
    float slow_side_dist_;
    double slow_speed_;
    double normal_speed_;
    float normal_dist_;
    float normal_side_dist_;
    double fast_speed_;
    float slow_min_x_;
    float slow_max_x_;
    float slow_min_y_;
    float slow_max_y_;
    float normal_speed_min_x_;
    float normal_speed_max_x_;
    float normal_speed_min_y_;
    float normal_speed_max_y_;

    float safety_offset_;
    float lidar_x_;

    float robot_length_half_x_;
    float robot_length_half_y_;

    float lidar_min_rad_;
    float lidar_max_rad_;
    float lidar_incr_rad_;
    float lidar_min_range_;

    size_t lidar_msg_range_size_;
    std::vector<float> lidar_msg_angles_;

    float cur_point_x_;
    float cur_point_y_;

protected:
    std::string robot_id_;
    std::string planner_setparam_srv_name_;

};

#endif // MANUALCONTROLLERS_H
