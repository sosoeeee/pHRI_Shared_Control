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
    ROS_INFO("Initializing the planner");
    // subscribe to FeedbackMsg
    

    // load ros parameters from node handle
    config.loadRosParamFromNodeHandle(nh);

    // Setup visualization
    visual = TebVisualizationPtr(new TebVisualization(nh, config));
    
    // Setup robot shape model
    RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(nh);
  
    // Setup planner (homotopy class planning or just the local teb planner)
    if (config.hcp.enable_homotopy_class_planning)
        planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
    else
        planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));

}

void TebLocalPlanner::loadObstacles()
{   
    ROS_INFO_ONCE("Obstacles loaded. This message is printed once.");
    // clear existing obstacles
    obst_vector.clear();
    // transform obstacles into the planning frame
    for (int i = 0; i < obstacles.markers.size(); i++)
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        polyobst->pushBackVertex(obstacles.markers[i].pose.position.x - obstacles.markers[i].scale.x / 2, obstacles.markers[i].pose.position.y - obstacles.markers[i].scale.y / 2);
        polyobst->pushBackVertex(obstacles.markers[i].pose.position.x + obstacles.markers[i].scale.x / 2, obstacles.markers[i].pose.position.y - obstacles.markers[i].scale.y / 2);
        polyobst->pushBackVertex(obstacles.markers[i].pose.position.x + obstacles.markers[i].scale.x / 2, obstacles.markers[i].pose.position.y + obstacles.markers[i].scale.y / 2);
        polyobst->pushBackVertex(obstacles.markers[i].pose.position.x - obstacles.markers[i].scale.x / 2, obstacles.markers[i].pose.position.y + obstacles.markers[i].scale.y / 2);
        polyobst->finalizePolygon();
        obst_vector.push_back(ObstaclePtr(polyobst)); //这里实际上是一个shared_ptr指向了创建的PolygonObstacle对象
    }
}

void TebLocalPlanner::loadViaPoints()
{
    ROS_INFO_ONCE("Via-points loaded. This message is printed once.");
    // clear existing via-points
    via_points.clear();
    // transform via-points into the planning frame
    for (int i = 0; i < ref_path.size(); i += start_pos.size())
    {
        via_points.emplace_back(ref_path[i], ref_path[i + 1]);
    }
}

void TebLocalPlanner::switchToMsg(const std::vector<TrajectoryPointMsg> &teb_trajectory, std::vector<float> &trajectory)
{
    // g(t) = f(kt) 时间尺度变换
    double teb_total_time = teb_trajectory.back().time_from_start.toSec();
    double K = teb_total_time / double(total_time);
    int N = std::floor(total_time * control_frequency);

    // start point
    trajectory.push_back(teb_trajectory[0].pose.position.x);
    trajectory.push_back(teb_trajectory[0].pose.position.y);
    trajectory.push_back(goal_pos[2]);
    trajectory.push_back(teb_trajectory[0].velocity.linear.x);
    trajectory.push_back(teb_trajectory[0].velocity.linear.y);
    trajectory.push_back(0);

    int idx = 1;
    double target_time = K * (idx * (1 / control_frequency));
    int idx_teb = 0;
    float t_s = 0;
    float t_e = 0;
    while (idx_teb < teb_trajectory.size() - 1 && idx < N - 1)
    {
        t_s = teb_trajectory[idx_teb].time_from_start.toSec();
        t_e = teb_trajectory[idx_teb + 1].time_from_start.toSec();
        if (t_s <= target_time && target_time < t_e)
        {
            float dt = t_s - target_time;
            float x = teb_trajectory[idx_teb].pose.position.x;
            float y = teb_trajectory[idx_teb].pose.position.y;
            float vx = teb_trajectory[idx_teb].velocity.linear.x;
            float vy = teb_trajectory[idx_teb].velocity.linear.y;

            trajectory.push_back(x + vx * dt);
            trajectory.push_back(y + vy * dt);
            trajectory.push_back(goal_pos[2]);
            trajectory.push_back(vx);
            trajectory.push_back(vy);
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
    trajectory.push_back(teb_trajectory.back().pose.position.x);
    trajectory.push_back(teb_trajectory.back().pose.position.y);
    trajectory.push_back(goal_pos[2]);
    trajectory.push_back(teb_trajectory.back().velocity.linear.x);
    trajectory.push_back(teb_trajectory.back().velocity.linear.y);
    trajectory.push_back(0);
}

void TebLocalPlanner::planTrajectory(std::vector<float> &trajectory)
{   
    ROS_INFO("Planning the trajectory");
    // update obstacles
    loadObstacles();
     
    // update via-points
    loadViaPoints();
    
    // Plan the trajectory
    // no robot posture info
    planner->plan(PoseSE2(start_pos[0], start_pos[1], 0), PoseSE2(goal_pos[0], goal_pos[1], 0)); 
    planner->visualize(); // pub FeedbackMsg to "teb_feedback"

    std::vector<TrajectoryPointMsg> teb_trajectory;
    planner->getFullTrajectory(teb_trajectory);

    std::vector<float> trajectory;
    switchToMsg(teb_trajectory, trajectory);
}