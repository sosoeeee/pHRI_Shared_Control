# include "ros/ros.h"
# include "local_planner/teb_local_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner");
    TebLocalPlanner teb_local_planner;
    teb_local_planner.run();

    return 0;
}