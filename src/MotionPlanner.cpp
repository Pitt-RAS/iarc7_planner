////////////////////////////////////////////////////////////////////////////
//
// MotionPlanner
//
// This is the top level class for the graph-search motion planner node. 
//
// Requests come into the planner via an action over an action server.
//
// Motion Point Stamped Arrays (plans) are returned back to the requester
//
////////////////////////////////////////////////////////////////////////////

// ROS headers
#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"

// Safety Client
#include "iarc7_safety/SafetyClient.hpp"

// message headers
#include "iarc7_msgs/PlanAction.h"
#include "iarc7_msgs/MotionPoint.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"

#include <planning_ros_utils/voxel_grid.h>
#include <ros/ros.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <motion_primitive_library/primitive/poly_solver.h>
#include <motion_primitive_library/planner/mp_map_util.h>

#include <iarc7_msgs/ObstacleArray.h>


using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Accel;

typedef actionlib::SimpleActionServer<iarc7_msgs::PlanAction> Server;

enum class PlannerState { WAITING,
                         PLANNING };

struct PlannerSettings
{
    // min and max arena limits
    double max_arena_limits[3];
    double min_arena_limits[3];

    // speed, acceleration, jerk limits
    double kinematic_constraints[3];

    // pos, vel, accel tolerances
    double tolerances[3];

    double dt;
    double eps;
    double u_max;

    int max_num;
    int num;
    int ndt;

    vec_Vec3f U;
};

bool checkGoal(Pose& pose_goal, Twist& twist_goal,
                                Accel& accel_goal,
                                double kinematic_constraints[3],
                                double max_arena[3],
                                double min_arena[3]) {
    if (pose_goal.position.x > max_arena[0]
        || pose_goal.position.y > max_arena[1]
        || pose_goal.position.z > max_arena[2]) {

        ROS_ERROR("Goal provided to planner is above arena limits");
        return false;
    }

    if (pose_goal.position.x < min_arena[0]
        || pose_goal.position.z < min_arena[1]
        || pose_goal.position.z < min_arena[2]) {

        ROS_ERROR("Goal provided to planner is below arena limits");
        return false;
    }
    
    if (std::max({std::abs(twist_goal.linear.x),
                  std::abs(twist_goal.linear.y),
                  std::abs(twist_goal.linear.z)}) > kinematic_constraints[0]) {

        ROS_ERROR("Goal provided to planner is beyond platform velocity limits");
        return false;
    }

    if (std::max({std::abs(accel_goal.linear.x),
                  std::abs(accel_goal.linear.y),
                  std::abs(accel_goal.linear.z)}) > kinematic_constraints[1]) {

        ROS_ERROR("Goal provided to planner is beyond platform acceleration limits");
        return false;
    }

    return true;
}

// Main entry point for the motion planner
int main(int argc, char **argv)
{
    // Required by ROS before calling many functions
    ros::init(argc, argv, "Motion_Planner");

    ROS_INFO("Motion_Planner begin");

    // Create a node handle for the node
    ros::NodeHandle nh;
    
    // This node handle has a specific namespace that allows us to easily
    // encapsulate parameters
    ros::NodeHandle private_nh ("~");

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
        ros::Duration(.005).sleep();
    }

    // Form a connection with the node monitor. If no connection can be made
    // assert because we don't know what's going on with the other nodes.
    ROS_INFO("motion_planner: Attempting to form safety bond");
    
    Iarc7Safety::SafetyClient safety_client(nh, "motion_planner");
    
    ROS_ASSERT_MSG(safety_client.formBond(),
                   "motion_planner: Could not form bond with safety client");

    // start action server
    Server server(nh, "planner_request", false);
    server.start();

    // LOAD PARAMETERS

    // node update frequency
    double update_frequency;

    // Update frequency retrieve
    ROS_ASSERT(private_nh.getParam("planner_update_frequency", update_frequency));

    // settings for planner
    PlannerSettings settings;

    // max x, y, z arena limits
    ROS_ASSERT(private_nh.getParam("/arena/max_x", settings.max_arena_limits[0]));
    ROS_ASSERT(private_nh.getParam("/arena/max_y", settings.max_arena_limits[1]));
    ROS_ASSERT(private_nh.getParam("/arena/max_z", settings.max_arena_limits[2]));

    // min x, y, z arena limits
    ROS_ASSERT(private_nh.getParam("/arena/min_x", settings.min_arena_limits[0]));
    ROS_ASSERT(private_nh.getParam("/arena/min_y", settings.min_arena_limits[1]));
    ROS_ASSERT(private_nh.getParam("/arena/min_z", settings.min_arena_limits[2]));

    // speed, acceleration, jerk limits
    ROS_ASSERT(private_nh.getParam("max_speed", settings.kinematic_constraints[0]));
    ROS_ASSERT(private_nh.getParam("max_acceleration", settings.kinematic_constraints[1]));
    ROS_ASSERT(private_nh.getParam("max_jerk", settings.kinematic_constraints[2]));

    ROS_ASSERT(private_nh.getParam("dt", settings.dt));
    ROS_ASSERT(private_nh.getParam("eps", settings.eps));
    ROS_ASSERT(private_nh.getParam("ndt", settings.ndt));

    ROS_ASSERT(private_nh.getParam("u_max", settings.u_max));
    ROS_ASSERT(private_nh.getParam("max_num",settings. max_num));
    ROS_ASSERT(private_nh.getParam("num", settings.num));

    // kinematic tolerances
    ROS_ASSERT(private_nh.getParam("p_tol", settings.tolerances[0]));
    ROS_ASSERT(private_nh.getParam("v_tol", settings.tolerances[1]));
    ROS_ASSERT(private_nh.getParam("a_tol", settings.tolerances[2]));

    vec_Vec3f U;
    const decimal_t du = settings.u_max / settings.num;
    for (decimal_t dx = -settings.u_max; dx <= settings.u_max; dx += du) {
        for (decimal_t dy = -settings.u_max; dy <= settings.u_max; dy += du) {
            for (decimal_t dz = -settings.u_max; dz <= settings.u_max; dz += du) {
                U.push_back(Vec3f(dx, dy, dz));
            }
        }
    }

    settings.U = U;

    // debug publishers
    ros::Publisher map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
    ros::Publisher refined_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("trajectory_refined", 1, true);

    // Cache the time
    ros::Time last_time = ros::Time::now();

    // update frequency of the node
    ros::Rate rate(update_frequency);

    PlannerState state = PlannerState::WAITING;

    iarc7_msgs::PlanGoalConstPtr goal_; 

    Pose pose_goal;
    Twist twist_goal;
    Accel accel_goal;

    // Run until ROS says we need to shutdown
    while (ros::ok()) {
        // Check the safety client before updating anything
        // there is no safety response for this node, so 
        // shut down. 
        ROS_ASSERT_MSG(!safety_client.isFatalActive(),
                       "motion_planner: fatal event from safety");

        ROS_ASSERT_MSG(!safety_client.isSafetyActive(),
                        "Motion_Planner shutdown due to safety active");

        // Get the time
        ros::Time current_time = ros::Time::now();

        // Make sure we have a new timestamp. This can be a problem with simulated
        // time that does not update with high precision.
        if (current_time > last_time) {
            last_time = current_time;
            
            // goal was canceled
            if (server.isPreemptRequested() && state == PlannerState::PLANNING) {
                server.setPreempted();
                ROS_INFO("Preempt requested. Current planning goal was canceled");
                state = PlannerState::WAITING;
            }

            // get a new goal from action server
            if (server.isNewGoalAvailable()) {
                if (server.isActive()) {
                    ROS_INFO("New goal received with current goal still running");
                }

                // planning goal
                goal_ = server.acceptNewGoal();

                pose_goal = goal_->goal.motion_point.pose;
                twist_goal = goal_->goal.motion_point.twist;
                accel_goal = goal_->goal.motion_point.accel;

                if (!checkGoal(pose_goal, twist_goal, accel_goal,
                                        settings.kinematic_constraints,
                                        settings.max_arena_limits,
                                        settings.min_arena_limits)) {
                    ROS_ERROR("Planner aborting requested gaol");
                    server.setAborted();
                    state = PlannerState::WAITING;
                } else {
                    state = PlannerState::PLANNING;
                    ROS_INFO("New goal accepted by planner");
                }
            }

            // able to generate a plan
            if (state == PlannerState::PLANNING) {
                
                iarc7_msgs::PlanResult result_;
                iarc7_msgs::PlanFeedback feedback_;

                // indicate that planning was successful or not
                bool success_ = true;
                
                // planning calls will go here



                result_.success = success_;
                
                server.publishFeedback(feedback_);
            }
        }

        // Handle all ROS callbacks
        ros::spinOnce();
        rate.sleep();
    }

    // All is good.
    return 0;
}
