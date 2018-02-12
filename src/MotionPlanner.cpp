////////////////////////////////////////////////////////////////////////////
//
// MotionPlanner
//
// This is the top level class for the graph-search motion planner node. 
//
// Requests come into the planner via an action over an action server.
//
// Trajectories (plans) are returned back to the requester
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include "actionlib/server/simple_action_server.h"

#include "iarc7_safety/SafetyClient.hpp"
#include "iarc7_msgs/PlanAction.h"

#include "planning_ros_msgs/Trajectory.h"

typedef actionlib::SimpleActionServer<iarc7_msgs::PlanAction> Server;


enum class PlannerState { WAITING,
                         PLANNING }; 

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

    // Update frequency retrieve
    private_nh.param("update_frequency", update_frequency, 60.0);

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
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
            
            // get a new goal from action server
            if (server.isNewGoalAvailable()) {
                if (server.isActive()) {
                    ROS_INFO("New goal received with current goal still running");
                }
                
                // planning goal
                goal_ = server.acceptNewGoal();
                state = PlannerState::PLANNING;
                ROS_INFO("New goal accepted by planner");
            }

            // goal was canceled
            if (server.isPreemptRequested()) {
                server.setPreempted();
                ROS_INFO("Preempt requested. Current planning goal was canceled");
                state = PlannerState::WAITING;
            }

            // able to generate a plan
            if (state == PlannerState::PLANNING) {
                
                iarc7_msgs::PlanResult result_;
                iarc7_msgs::PlanFeedback feedback_;

                // indicate that planning was successful or not
                bool success_ = true;
                
                // planning here 
                
                result_.success = success_;
                feedback_.plan.header.frame_id = "/frame_id";
                
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
