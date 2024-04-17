# ifndef TEB_LOCAL_PLANNER_H_
# define TEB_LOCAL_PLANNER_H_

# include "local_planner/base_local_planner.h"
#include <teb_local_planner/teb_local_planner_ros.h>

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
public:
    TebLocalPlanner();
    ~TebLocalPlanner();
    void loadObstacles();
    void loadViaPoints();
    void switchToMsg(const std::vector<TrajectoryPointMsg> &teb_trajectory, std::vector<float> &trajectory);
    // Implement the virtual functions
    void initPlanner();
    void planTrajectory(std::vector<float> &trajectory);
};

#endif