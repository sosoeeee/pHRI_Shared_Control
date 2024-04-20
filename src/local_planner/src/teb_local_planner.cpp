#include "teb_local_planner.h"

TebLocalPlanner::TebLocalPlanner()
{
}

TebLocalPlanner::~TebLocalPlanner()
{
}

void TebLocalPlanner::initPlanner()
{
    // Initialize the planner
    ROS_INFO("Initializing the TebLocalPlanner");
    nh.getParam("/local_planner/spacial_scale", spacial_scale);
    nh.getParam("/local_planner/plan_times", plan_times);
    // subscribe to FeedbackMsg
    feedback_sub = nh.subscribe("/local_planner/teb_feedback", 10, &TebLocalPlanner::feedback_cb, this);

    dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlanner::CB_reconfigure, this, _1, _2);
    dynamic_recfg->setCallback(cb);

    // load ros parameters from node handle
    config.loadRosParamFromNodeHandle(nh);

    // Setup visualization
    visual = TebVisualizationPtr(new TebVisualization(nh, config));
    
    // Setup robot shape model
    robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(nh);
  
    // Setup planner (homotopy class planning or just the local teb planner)
    if (config.hcp.enable_homotopy_class_planning)
        planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
    else
        planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));

}

void TebLocalPlanner::CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
    config.reconfigure(reconfig);
}

void TebLocalPlanner::feedback_cb(const teb_local_planner::FeedbackMsg::ConstPtr &feedback)
{
    // do something with the feedback
    // ROS_INFO("Trajecory feedback received");
    teb_trajectory = feedback->trajectories[feedback->selected_trajectory_idx]; // deep copy
    is_plan_success = true;
}

void TebLocalPlanner::loadObstacles()
{   
    // clear existing obstacles
    obst_vector.clear();
    // transform obstacles into the planning frame
    for (int i = 0; i < obstacles.markers.size(); i++)
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        polyobst->pushBackVertex((obstacles.markers[i].pose.position.x - obstacles.markers[i].scale.x / 2) * spacial_scale, (obstacles.markers[i].pose.position.y - obstacles.markers[i].scale.y / 2) * spacial_scale);
        polyobst->pushBackVertex((obstacles.markers[i].pose.position.x + obstacles.markers[i].scale.x / 2) * spacial_scale, (obstacles.markers[i].pose.position.y - obstacles.markers[i].scale.y / 2) * spacial_scale);
        polyobst->pushBackVertex((obstacles.markers[i].pose.position.x + obstacles.markers[i].scale.x / 2) * spacial_scale, (obstacles.markers[i].pose.position.y + obstacles.markers[i].scale.y / 2) * spacial_scale);
        polyobst->pushBackVertex((obstacles.markers[i].pose.position.x - obstacles.markers[i].scale.x / 2) * spacial_scale, (obstacles.markers[i].pose.position.y + obstacles.markers[i].scale.y / 2) * spacial_scale);
        polyobst->finalizePolygon();
        obst_vector.push_back(ObstaclePtr(polyobst)); //这里实际上是一个shared_ptr指向了创建的PolygonObstacle对象
    }
    // ROS_INFO_ONCE("Obstacles loaded. This message is printed once.");
}

void TebLocalPlanner::loadViaPoints()
{
    // clear existing via-points
    via_points.clear();
    init_plan.clear();
    // transform via-points into the planning frame
    for (int i = 0; i < ref_path.size(); i += start_pos.size())
    {
        if (i >= start_pos.size() && i < ref_path.size() - start_pos.size())
            via_points.emplace_back(ref_path[i] * spacial_scale, ref_path[i + 1] * spacial_scale);

        //add init plan
        geometry_msgs::PoseStamped initPose;
        initPose.pose.position.x = ref_path[i] * spacial_scale;
        initPose.pose.position.y = ref_path[i + 1] * spacial_scale;
        initPose.pose.position.z = 0;
        initPose.pose.orientation.x = 0;
        initPose.pose.orientation.y = 0;
        initPose.pose.orientation.z = 0;
        initPose.pose.orientation.w = 1;
        init_plan.push_back(initPose);
    }

    //debug
    // via_points.emplace_back(0.4501 * spacial_scale, -0.4027 * spacial_scale);
    // via_points.emplace_back(0.5279 * spacial_scale, -0.4067 * spacial_scale);
    // via_points.emplace_back(0.5968 * spacial_scale, -0.4024 * spacial_scale);
    // via_points.emplace_back(0.6251 * spacial_scale, -0.3461 * spacial_scale);
    // via_points.emplace_back(0.6217 * spacial_scale, -0.2696 * spacial_scale);
    // via_points.emplace_back(0.5643 * spacial_scale, -0.2361 * spacial_scale);
    // via_points.emplace_back(0.4994 * spacial_scale, -0.2067 * spacial_scale);

    // ROS_INFO_ONCE("Via-points loaded. This message is printed once.");
}

void TebLocalPlanner::switchToMsg(const TrajectoryMsg &teb_trajectory, std::vector<float> &trajectory)
{
    trajectory.clear();
    // g(t) = f(kt) 时间尺度变换
    double teb_total_time = teb_trajectory.trajectory.back().time_from_start.toSec();
    // ROS_INFO("TEB time: %f", teb_total_time);
    // ROS_INFO("Target time: %f", total_time);
    double K = teb_total_time / double(total_time);
    int N = std::floor(total_time * control_frequency);

    // start point
    trajectory.push_back(teb_trajectory.trajectory[0].pose.position.x / spacial_scale);
    trajectory.push_back(teb_trajectory.trajectory[0].pose.position.y / spacial_scale);
    trajectory.push_back(goal_pos[2]);
    trajectory.push_back(teb_trajectory.trajectory[0].velocity.linear.x / spacial_scale);
    trajectory.push_back(teb_trajectory.trajectory[0].velocity.linear.y / spacial_scale);
    trajectory.push_back(0);

    int idx = 1;
    double target_time = K * (idx * (1 / control_frequency));
    int idx_teb = 0;
    float t_s = 0;
    float t_e = 0;
    while (idx_teb < teb_trajectory.trajectory.size() - 1 && idx < N - 1)
    {
        t_s = teb_trajectory.trajectory[idx_teb].time_from_start.toSec();
        t_e = teb_trajectory.trajectory[idx_teb + 1].time_from_start.toSec();
        if (t_s <= target_time && target_time < t_e)
        {
            // ROS_INFO("Sample between [%f, %f], at %f", t_s, t_e, target_time);
            float ratio = (target_time - t_s)/(t_e - t_s);
            float x_s = teb_trajectory.trajectory[idx_teb].pose.position.x / spacial_scale;
            float y_s = teb_trajectory.trajectory[idx_teb].pose.position.y / spacial_scale;
            float x_e = teb_trajectory.trajectory[idx_teb+1].pose.position.x / spacial_scale;
            float y_e = teb_trajectory.trajectory[idx_teb+1].pose.position.y / spacial_scale;

            trajectory.push_back(x_s + (x_e - x_s) * ratio);
            trajectory.push_back(y_s + (y_e - y_s) * ratio);
            trajectory.push_back(goal_pos[2]);
            trajectory.push_back((x_e - x_s)/(t_e - t_s));
            trajectory.push_back((y_e - y_s)/(t_e - t_s));
            trajectory.push_back(0);
            idx++;
            target_time = K * (idx * (1 / control_frequency));
        }
        else
        {
            idx_teb++;   
        }
    }

    // end point
    trajectory.push_back(teb_trajectory.trajectory.back().pose.position.x / spacial_scale);
    trajectory.push_back(teb_trajectory.trajectory.back().pose.position.y / spacial_scale);
    trajectory.push_back(goal_pos[2]);
    trajectory.push_back(teb_trajectory.trajectory.back().velocity.linear.x / spacial_scale);
    trajectory.push_back(teb_trajectory.trajectory.back().velocity.linear.y / spacial_scale);
    trajectory.push_back(0);
}

void TebLocalPlanner::planTrajectory(std::vector<float> &trajectory)
{   
    // ROS_INFO("Planning the trajectory");
    // update obstacles
    loadObstacles();
     
    // update via-points
    loadViaPoints();
    
    // Plan the trajectory
    // no robot posture info
    // ROS_INFO("START [%f, %f, %f]", start_pos[0], start_pos[1], start_pos[2]);
    // ROS_INFO("GOAL [%f, %f, %f]", goal_pos[0], goal_pos[1], goal_pos[2]);

    // debug to adjust parameters
    // while (true)
    // {
    //     planner->plan(PoseSE2(start_pos[0] * spacial_scale, start_pos[1] * spacial_scale, 0), PoseSE2(goal_pos[0] * spacial_scale, goal_pos[1] * spacial_scale, 0));
    //     visual->publishObstacles(obst_vector);
    //     visual->publishViaPoints(via_points);
    //     planner->visualize(); // pub FeedbackMsg to "teb_feedback"
    //     ros::Duration(0.1).sleep();
    // }

    is_plan_success = false;
    for (int i = 0; i<plan_times; i++)
    {
        planner->plan(init_plan);
        // visual->publishObstacles(obst_vector);
        // visual->publishViaPoints(via_points);
        // planner->visualize(); // pub FeedbackMsg to "teb_feedback"
        // ros::Duration(0.1).sleep();
    }
    visual->publishObstacles(obst_vector);
    visual->publishViaPoints(via_points);
    planner->visualize(); // pub FeedbackMsg to "teb_feedback"

    while (!is_plan_success)
    {
        ROS_INFO("Wait Feedback info");
        ros::Duration(0.01).sleep();
    }    
    // switch to the format of the output
    switchToMsg(teb_trajectory, trajectory);
}