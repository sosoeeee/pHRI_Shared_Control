#ifndef BASE_LOCAL_PLANNER_H_
#define BASE_LOCAL_PLANNER_H_

// BASE_LOCAL_PLANNER_H_

#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "local_planner/LocalPlanning.h"
#include "global_planner/GlobalPlanning.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"

class BaseLocalPlanner
{
protected:
    // local planner variables
    std::vector<float> start_pos;
    std::vector<float> start_vel;
    std::vector<float> start_acc;
    std::vector<float> goal_pos;
    std::vector<float> goal_vel;
    std::vector<float> goal_acc;
    float max_vel;
    float max_acc;
    float total_time;
    float control_frequency;

    // global planner variables
    std::vector<float> ref_path;
    bool global_plan_success;

    // obstacle variables
    visualization_msgs::MarkerArray obstacles;
    tf::TransformListener listener;
    std::string target_frame;

    ros::NodeHandle nh;
public:
    BaseLocalPlanner();
    ~BaseLocalPlanner();
    void global_planner_client();
    void run();
    virtual void initPlanner()=0;
    virtual void planTrajectory(std::vector<float> &trajectory)=0;
    bool local_plan_cb(local_planner::LocalPlanning::Request &req, local_planner::LocalPlanning::Response &res);
};

BaseLocalPlanner::BaseLocalPlanner()
{
    nh = ros::NodeHandle("~");
    // Initialize the service server
    ros::ServiceServer service = nh.advertiseService("local_plan", &BaseLocalPlanner::local_plan_cb, this);

    // subscribe to environment information
    ros::Subscriber sub = nh.subscribe("/env_obstacles", 1, &BaseLocalPlanner::obstacle_cb, this);
    nh.getParam('/world_frame', target_frame);
}

BaseLocalPlanner::obstacle_cb(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    // transform the obstacles to the target frame
    // ATTENTION: after transforming, we only change the pose of each marker in MarkerArray, but the header frame_id is still the original frame_id
    obstacles.markers.clear();
    for (int i = 0; i < msg->markers.size(); i++)
    {
        visualization_msgs::Marker marker = msg->markers[i];
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header = marker.header;
        pose_in.pose = marker.pose;
        listener.transformPose(target_frame, pose_in, pose_out);
        marker.pose = pose_out.pose;
        obstacles.markers.push_back(marker);
    }
}

BaseLocalPlanner::local_plan_cb(local_planner::LocalPlanning::Request &req, local_planner::LocalPlanning::Response &res)
{
    // Get information from the request
    start_pos.assign(req.start_pos.begin(), req.start_pos.end());
    start_vel.assign(req.start_vel.begin(), req.start_vel.end());
    start_acc.assign(req.start_acc.begin(), req.start_acc.end());
    goal_pos.assign(req.goal_pos.begin(), req.goal_pos.end());
    goal_vel.assign(req.goal_vel.begin(), req.goal_vel.end());
    goal_acc.assign(req.goal_acc.begin(), req.goal_acc.end());

    max_vel = req.max_vel;
    max_acc = req.max_acc;
    total_time = req.total_time;
    control_frequency = req.control_frequency;

    // Call the global planner
    global_planner_client();

    // Plan the trajectory
    if (global_plan_success)
    {   
        std::vector<float> traj;
        planTrajectory(traj);
        res.trajectory.assign(traj.begin(), traj.end());
        res.success = true;
    }
    else
    {
        res.success = false;
    }

    return true;
}

BaseLocalPlanner::global_planner_client()
{
    // Initialize the client
    ros::service::waitForService("global_plan");
    ros::ServiceClient client = nh.serviceClient<global_planner::GlobalPlanning>("global_plan");
    global_planner::GlobalPlanning srv;
    srv.request.start = start_pos;
    srv.request.goal = goal_pos;

    if (client.call(srv))
    {
        global_plan_success = srv.response.success;
        if (global_plan_success)
        {
            ref_path.assign(srv.response.path.begin(), srv.response.path.end());
        }
        else
        {
            ROS_ERROR("Failed to get global plan");
        }
    }
    else
    {
        ROS_ERROR("Failed to call service global_planner");
    }
}

BaseLocalPlanner::run()
{
    initPlanner();
    ros::spin();
}

BaseLocalPlanner::~BaseLocalPlanner()
{
}


#endif