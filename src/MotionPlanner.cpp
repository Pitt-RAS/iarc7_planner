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

typedef actionlib::SimpleActionServer<iarc7_msgs::PlanAction> Server;

enum class PlannerState { WAITING,
                         PLANNING }; 

bool checkGoal(iarc7_msgs::PlanGoalConstPtr goal_, double kinematic_constraints[3], 
                                                    double max_arena[3], 
                                                    double min_arena[3]) {
    if (goal_->x_pos_goal > max_arena[0]
        || goal_->y_pos_goal > max_arena[1]
        || goal_->z_pos_goal > max_arena[2]){

        ROS_ERROR("Goal provided to planner is above arena limits");
        return false;
    }

    if (goal_->x_pos_goal < min_arena[0]
        || goal_->y_pos_goal < min_arena[1]
        || goal_->z_pos_goal < min_arena[2]){

        ROS_ERROR("Goal provided to planner is below arena limits");
        return false;
    }
    
    if (std::max({goal_->x_vel_goal, 
                  goal_->y_vel_goal, 
                  goal_->z_vel_goal}) > kinematic_constraints[0]) {

        ROS_ERROR("Goal provided to planner is beyond platform velocity limits");
        return false;
    }

    if (std::max({goal_->x_accel_goal, 
                  goal_->y_accel_goal, 
                  goal_->z_accel_goal}) > kinematic_constraints[1]) {

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

    // LOAD PARAMETERS
    double update_frequency;

    // speed, acceleration, jerk limits
    double kinematic_constraints[3];

    // max x, y, z arena limits
    double max_arena_limits[3];

    // min x, y, z arena limits
    double min_arena_limits[3];

    // Update frequency retrieve
    private_nh.param("planner_update_frequency", update_frequency, 60.0);

    // kinematic constraints retrieve
    private_nh.param("max_speed", kinematic_constraints[0], 3.0);
    private_nh.param("max_acceleration", kinematic_constraints[1], 5.0);
    private_nh.param("max_jerk", kinematic_constraints[2], 10.0);

    // arena size/limits retrieve
    private_nh.param("/arena/max_x", max_arena_limits[0], 3.0);
    private_nh.param("/arena/max_y", max_arena_limits[1], 3.0);
    private_nh.param("/arena/max_z", max_arena_limits[2], 3.0);
    
    private_nh.param("/arena/min_x", min_arena_limits[0], 0.0);
    private_nh.param("/arena/min_y", min_arena_limits[1], 0.0);
    private_nh.param("/arena/min_z", min_arena_limits[2], 0.0);

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
        ros::Duration(.005).sleep();
    }

    Server server(nh, "planner_request", false);
    server.start();

    // Form a connection with the node monitor. If no connection can be made
    // assert because we don't know what's going on with the other nodes.
    ROS_INFO("motion_planner: Attempting to form safety bond");
    
    Iarc7Safety::SafetyClient safety_client(nh, "motion_planner");
    
    ROS_ASSERT_MSG(safety_client.formBond(),
                   "motion_planner: Could not form bond with safety client");

    // Cache the time
    ros::Time last_time = ros::Time::now();

    // update frequency of the node
    ros::Rate rate(update_frequency);

    PlannerState state = PlannerState::WAITING;

    iarc7_msgs::PlanGoalConstPtr goal_; 

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

                if (!checkGoal(goal_, kinematic_constraints, 
                                        max_arena_limits,
                                        min_arena_limits)) {
                    
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
