////////////////////////////////////////////////////////////////////////////
//
// Map Generator
//
// Generates maps from obstacle information
//
////////////////////////////////////////////////////////////////////////////

#ifndef MAP_GENERATOR_H
#define MAP_GENERATOR_H

#include <ros/ros.h>

// ROS message headers
#include "iarc7_msgs/MotionPointStamped.h"
#include "iarc7_msgs/ObstacleArray.h"

#include <planning_ros_utils/voxel_grid.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <motion_primitive_library/planner/mp_map_util.h>

#include <iarc7_msgs/ObstacleArray.h>
namespace Iarc7Planner
{

class MapGenerator
{
public:
    MapGenerator(ros::NodeHandle nh);

    getMap()

private:
};

} // End namespace Iarc7Planner

#endif // MAP_GENERATOR_H
