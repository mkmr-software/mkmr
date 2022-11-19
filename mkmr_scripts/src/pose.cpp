#include "ros/ros.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/StartTrajectory.h"

ros::ServiceClient get_trajectory_states_client;
ros::ServiceClient finish_trajectory_client;
ros::ServiceClient start_trajectory_client;

std::string configuration_directory;
std::string configuration_basename;

void setInitialPose(geometry_msgs::Pose initial_pose)
{
    // Get trajectories
    cartographer_ros_msgs::GetTrajectoryStates get_trajectory_states_srv;
    if(!get_trajectory_states_client.call(get_trajectory_states_srv))
    {
        ROS_ERROR("Could not get trajectory states");
        return;
    }

    // Finish current trajectory
    int trajectory_count = get_trajectory_states_srv.response.trajectory_states.trajectory_id.size();
    int current_trajectory_id = get_trajectory_states_srv.response.trajectory_states.trajectory_id[trajectory_count - 1];

    cartographer_ros_msgs::FinishTrajectory finish_trajectory_srv;
    finish_trajectory_srv.request.trajectory_id = current_trajectory_id;
    if(!finish_trajectory_client.call(finish_trajectory_srv))
    {
        ROS_ERROR("Could not finish trajectory");
        return;
    }

    // Start new trajectory with given initial pose
    cartographer_ros_msgs::StartTrajectory start_trajectory_srv;

    start_trajectory_srv.request.configuration_directory = configuration_directory;
    start_trajectory_srv.request.configuration_basename = configuration_basename;

    start_trajectory_srv.request.use_initial_pose = true;
    start_trajectory_srv.request.initial_pose = initial_pose;

    if(!start_trajectory_client.call(start_trajectory_srv))
    {
        ROS_ERROR("Could not start new trajectory");
        return;
    }

    ROS_ERROR("Initial Pose Given Successfuly");
    std::cout << "X: " << initial_pose.position.x << ", Y: " << initial_pose.position.y << std::endl;
}

void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    setInitialPose(msg->pose.pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose");
    ros::NodeHandle n, n_priv("~");

    // Lua file parameters
    if(!n_priv.getParam("configuration_directory", configuration_directory)) configuration_directory = "";
    if(!n_priv.getParam("configuration_basename", configuration_basename)) configuration_basename = "localization.lua";

    // Initial pose parameters
    double initial_pose_x, initial_pose_y, initial_yaw;
    if(!n_priv.getParam("initial_pose_x", initial_pose_x)) initial_pose_x = 0.0;
    if(!n_priv.getParam("initial_pose_y", initial_pose_y)) initial_pose_y = 0.0;
    if(!n_priv.getParam("initial_yaw", initial_yaw)) initial_yaw = 0.0;

    // Pose publisher parameters
    bool publish_pose_topic;
    std::string map_frame, base_frame;
    if(!n_priv.getParam("publish_pose_topic", publish_pose_topic)) publish_pose_topic = false;
    if(!n_priv.getParam("map_frame", map_frame)) map_frame = "map";
    if(!n_priv.getParam("base_frame", base_frame)) base_frame = "base_footprint";

    ros::Subscriber initial_pose_sub = n.subscribe("initialpose", 1, initial_pose_cb);

    // Wait for Cartographer services
    // if(!ros::service::waitForService("get_trajectory_states", 5000) ||
    //         !ros::service::waitForService("finish_trajectory", 5000) ||
    //         !ros::service::waitForService("start_trajectory", 5000))
    // {
    //     ROS_ERROR("Cartographer services not available");
    //     return 1;
    // }

    // ROS Service Clients
    get_trajectory_states_client = n.serviceClient<cartographer_ros_msgs::GetTrajectoryStates>("get_trajectory_states");
    finish_trajectory_client = n.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    start_trajectory_client = n.serviceClient<cartographer_ros_msgs::StartTrajectory>("start_trajectory");

    // Set first initial pose
    if(initial_pose_x != 0.0 && initial_pose_y != 0.0 && initial_yaw != 0.0)
    {
        geometry_msgs::Pose initial_pose;

        initial_pose.position.x = initial_pose_x;
        initial_pose.position.y = initial_pose_y;
        initial_pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, initial_yaw);
        initial_pose.orientation = tf2::toMsg(q);

        setInitialPose(initial_pose);
    }

    if(!publish_pose_topic) ros::spin();
    else
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener transform_listener(tfBuffer);

        geometry_msgs::TransformStamped map_transform;
        geometry_msgs::PoseWithCovarianceStamped pose;

        ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);

        pose.header.frame_id = map_frame;

        ros::Rate rate(50);
        while(ros::ok())
        {
            ros::spinOnce();
            rate.sleep();

            try
            {
                map_transform = tfBuffer.lookupTransform(map_frame, base_frame, ros::Time(0));

                pose.header.stamp = ros::Time::now();

                pose.pose.pose.position.x = map_transform.transform.translation.x;
                pose.pose.pose.position.y = map_transform.transform.translation.y;
                pose.pose.pose.position.z = map_transform.transform.translation.z;

                pose.pose.pose.orientation.x = map_transform.transform.rotation.x;
                pose.pose.pose.orientation.y = map_transform.transform.rotation.y;
                pose.pose.pose.orientation.z = map_transform.transform.rotation.z;
                pose.pose.pose.orientation.w = map_transform.transform.rotation.w;

                pose_pub.publish(pose);
            }
            catch (tf2::TransformException ex)
            {
                continue;
            }
        }
    }
}