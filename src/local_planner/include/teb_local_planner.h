# ifndef TEB_LOCAL_PLANNER_H_
# define TEB_LOCAL_PLANNER_H_

# include "base_local_planner.h"
#include <teb_local_planner/teb_local_planner_ros.h>
#include <teb_local_planner/TrajectoryMsg.h>
#include <teb_local_planner/FeedbackMsg.h>

using namespace teb_local_planner;

class TebLocalPlanner : public BaseLocalPlanner
{
private:
    // teb local planner
    PlannerInterfacePtr planner;
    TebVisualizationPtr visual;
    std::vector<ObstaclePtr> obst_vector;
    ViaPointContainer via_points;
    TebConfig config;
    RobotFootprintModelPtr robot_model;
    bool is_plan_success;
    TrajectoryMsg teb_trajectory;
    ros::Subscriber feedback_sub;

    // spacial scale
    float spacial_scale;
public:
    TebLocalPlanner();
    ~TebLocalPlanner();
    void loadObstacles();
    void loadViaPoints();
    void switchToMsg(const TrajectoryMsg &teb_trajectory, std::vector<float> &trajectory);
    void feedback_cb(const teb_local_planner::FeedbackMsg::ConstPtr &feedback);
    // Implement the virtual functions
    void initPlanner();
    void planTrajectory(std::vector<float> &trajectory);
};

#endif