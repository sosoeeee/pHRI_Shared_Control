# ifndef TEB_LOCAL_PLANNER_H_
# define TEB_LOCAL_PLANNER_H_

# include "base_local_planner.h"
#include <teb_local_planner/teb_local_planner_ros.h>
#include <teb_local_planner/TrajectoryMsg.h>
#include <teb_local_planner/FeedbackMsg.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

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
    boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
public:
    TebLocalPlanner();
    ~TebLocalPlanner();
    void loadObstacles();
    void loadViaPoints();
    void switchToMsg(const TrajectoryMsg &teb_trajectory, std::vector<float> &trajectory);
    void feedback_cb(const teb_local_planner::FeedbackMsg::ConstPtr &feedback);
    void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
    // Implement the virtual functions
    void initPlanner();
    void planTrajectory(std::vector<float> &trajectory);
};

#endif