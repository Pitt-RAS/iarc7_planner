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
#include "actionlib/server/simple_action_server.h"
#include <ros/ros.h>

// Safety Client
#include "iarc7_safety/SafetyClient.hpp"

// ROS message headers
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

#include "nav_msgs/Odometry.h"

// MPL message headers
#include <planning_ros_msgs/VoxelMap.h>

#include <iarc7_msgs/MotionPointStampedArray.h>
#include <iarc7_msgs/MotionPoint.h>
#include <iarc7_msgs/PlanAction.h>
#include <iarc7_msgs/ObstacleArray.h>

#include <motion_primitive_library/planner/mp_map_util.h>

#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <planning_ros_utils/voxel_grid.h>

using iarc7_msgs::MotionPointStamped;
using geometry_msgs::Accel;
using geometry_msgs::Pose;
using geometry_msgs::Twist;
using geometry_msgs::Vector3;
using geometry_msgs::Point;

typedef actionlib::SimpleActionServer<iarc7_msgs::PlanAction> Server;

bool checkGoal(Point &pose_goal, Vector3 &twist_goal, Vector3 &accel_goal,
               double kinematic_constraints[3],
               double max_arena[3],
               double min_arena[3]) {
    if (pose_goal.x > max_arena[0] ||
        pose_goal.y > max_arena[1] ||
        pose_goal.z > max_arena[2]) {
        ROS_ERROR("Goal provided to planner is above arena limits");
        return false;
    }

    if (pose_goal.x < min_arena[0] ||
        pose_goal.z < min_arena[1] ||
        pose_goal.z < min_arena[2]) {
        ROS_ERROR("Goal provided to planner is below arena limits");
        return false;
    }

    if (std::max({std::abs(twist_goal.x), std::abs(twist_goal.y),
                  std::abs(twist_goal.z)}) > kinematic_constraints[0]) {
        ROS_ERROR(
            "Goal provided to planner is beyond platform velocity limits");
        return false;
    }

    if (std::max({std::abs(accel_goal.x), std::abs(accel_goal.y),
                  std::abs(accel_goal.z)}) > kinematic_constraints[1]) {
        ROS_ERROR("Goal provided to planner is beyond platform acceleration limits");
        return false;
    }

    return true;
}

// Main entry point for the motion planner
int main(int argc, char **argv) {
    // Required by ROS before calling many functions
    ros::init(argc, argv, "Motion_Planner");

    ROS_INFO("Motion Planner begin");

    // Create a node handle for the node
    ros::NodeHandle nh;

    // This node handle has a specific namespace that allows us to easily
    // encapsulate parameters
    ros::NodeHandle private_nh("~");

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
        ros::Duration(.005).sleep();
    }

   // start action server
    Server server(nh, "planner_request", false);
    server.start();

    /******** LOAD PARAMETERS ********/

    // node update frequency
    double update_frequency;

    // max total planning time
    double max_planning_time;

    // radius of drone in meters
    double drone_radius;

    // safety buffer to stay away from obstacles
    double obst_buffer;

    /******** SETTINGS FOR PLANNER ********/

    // min and max arena limits
    double max_arena_limits[3];
    double min_arena_limits[3];

    // speed, acceleration, jerk limits
    double kinematic_constraints[3];

    // tolerances
    double pose_tol;
    double vel_tol;
    double accel_tol;

    // min and max time discretization
    double min_dt;
    double max_dt;

    // max time steps
    int ndt;

    // greedy param, epsilon
    double eps;

    // max for control space
    double u_max;

    // resolution to generate map at
    double map_res;

    // samples per sec for generating waypoints
    double samples_per_sec;

    // max num of expansions
    int max_num;

    // control discretization
    int num;

    // controls which space to plan in
    bool use_pos;
    bool use_vel;
    bool use_acc;
    bool use_jrk;

    // control space
    vec_Vec3f U;

    // Update frequency retrieve
    private_nh.param("planner_update_frequency", update_frequency, 50.0);

    // max total planning time
    private_nh.param("planning_timeout", max_planning_time, 0.150);

    // used for obstacle dilation
    private_nh.param("radius", drone_radius, 1.0);
    private_nh.param("buffer", obst_buffer, 1.0);

    // max x, y, z arena limits
    private_nh.param("/arena/max_x", max_arena_limits[0], 0.0);
    private_nh.param("/arena/max_y", max_arena_limits[1], 0.0);
    private_nh.param("/arena/max_z", max_arena_limits[2], 0.0);

    // min x, y, z arena limits
    private_nh.param("/arena/min_x", min_arena_limits[0], 0.0);
    private_nh.param("/arena/min_y", min_arena_limits[1], 0.0);
    private_nh.param("/arena/min_z", min_arena_limits[2], 0.0);

    // resolution to generate map at
    private_nh.param("map_res", map_res, .25);

    // speed, acceleration, jerk limits
    private_nh.param("max_speed", kinematic_constraints[0], 0.0);
    private_nh.param("max_acceleration", kinematic_constraints[1], 0.0);
    private_nh.param("max_jerk", kinematic_constraints[2], 0.0);

    // min/max time discretization for each primitive
    private_nh.param("min_dt", min_dt, 0.05);
    private_nh.param("max_dt", max_dt, 0.15);

    private_nh.param("use_pos", use_pos, true);
    private_nh.param("use_vel", use_vel, true);
    private_nh.param("use_acc", use_acc, true);
    private_nh.param("use_jrk", use_jrk, false);

    // ndt*dt is the max plan length, in seconds
    private_nh.param("ndt", ndt, 1000);

    // Epsilon: greedy param
    private_nh.param("eps", eps, 10.0);

    // control discretization
    private_nh.param("num", num, 1);

    // maximum number of allowed expansion
    private_nh.param("max_num", max_num, 1000);

    // tolerances
    private_nh.param("p_tol", pose_tol, .075);
    private_nh.param("vel_tol", vel_tol, .1);
    private_nh.param("accel_tol", accel_tol, .15);

    // number of samples per second to sample trajectory at
    private_nh.param("samples_per_sec", samples_per_sec, 50.0);

    // max jerk is u max
    u_max = kinematic_constraints[2];

    // final obstacle dilation buffer
    double buffer = drone_radius + obst_buffer;

    const decimal_t du = u_max / num;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du) {
        for (decimal_t dy = -u_max; dy <= u_max; dy += du) {
            for (decimal_t dz = -u_max; dz <= u_max; dz += du) {
                U.push_back(Vec3f(dx, dy, dz));
            }
        }
    }

    iarc7_msgs::PlanGoalConstPtr goal_;

    Point pose_goal, pose_start;
    Vector3 twist_goal, twist_start;
    Vector3 accel_goal, accel_start;

    // subscribe to obstacle topic
    boost::shared_ptr<const iarc7_msgs::ObstacleArray> last_msg;
    boost::function<void(const boost::shared_ptr<const iarc7_msgs::ObstacleArray>&)> obstacle_callback =
        [&](const boost::shared_ptr<const iarc7_msgs::ObstacleArray>& msg) -> void {
            if (last_msg == nullptr || last_msg->header.stamp < msg->header.stamp) {
                last_msg = msg;
            } else {
                ROS_ERROR("Motion Planner: bad stamp on obstacle message");
            }
        };
    ros::Subscriber obstacle_sub = nh.subscribe("obstacles", 2, obstacle_callback);

    // Form a connection with the node monitor. If no connection can be made
    // assert because we don't know what's going on with the other nodes.
    ROS_INFO("Motion_Planner: Attempting to form safety bond");

    Iarc7Safety::SafetyClient safety_client(nh, "motion_planner");

    ROS_ASSERT_MSG(safety_client.formBond(),"Motion_Planner: Could not form bond with safety client");

    // node update rate
    ros::Rate rate(update_frequency);

    // Run until ROS says we need to shutdown
    while (ros::ok()) {
        // Check the safety client before updating anything
        // there is no safety response for this node, so
        // shut down.
        ROS_ASSERT_MSG(!safety_client.isFatalActive(), "MotionPlanner: fatal event from safety");
        ROS_ASSERT_MSG(!safety_client.isSafetyActive(), "MotionPlanner shutdown due to safety active");

        // goal was canceled
        if (server.isPreemptRequested()) {
            server.setPreempted();
            ROS_INFO("Preempt requested. Current planning goal was canceled");
        }

        if (last_msg == nullptr) {
             ROS_DEBUG_THROTTLE(30,"MotionPlanner: No obstacle information available, can't build map");
        } else {
            // get a new goal from action server
            if (server.isNewGoalAvailable()) {
                if (server.isActive()) {
                    ROS_ERROR("New goal received with current goal still running");
                }
                // planning goal
                goal_ = server.acceptNewGoal();

                // offset by start pose so that we never "go out" of arena
                double x_offset = 10;
                double y_offset = 10;

                pose_start.x = goal_->start.motion_point.pose.position.x + x_offset;
                pose_start.y = goal_->start.motion_point.pose.position.y + y_offset;
                pose_start.z = goal_->start.motion_point.pose.position.z;

                twist_start = goal_->start.motion_point.twist.linear;
                accel_start = goal_->start.motion_point.accel.linear;

                pose_goal.x = goal_->goal.motion_point.pose.position.x + x_offset;
                pose_goal.y = goal_->goal.motion_point.pose.position.y + y_offset;
                pose_goal.z = goal_->goal.motion_point.pose.position.z;

                twist_goal = goal_->goal.motion_point.twist.linear;
                accel_goal = goal_->goal.motion_point.accel.linear;

                if (!checkGoal(pose_goal, twist_goal, accel_goal,
                               kinematic_constraints,
                               max_arena_limits,
                               min_arena_limits)) {
                    ROS_ERROR("Planner aborting requested goal");

                    iarc7_msgs::PlanResult result_;
                    result_.success = false;
                    server.setAborted(result_);
                } else {
                    ROS_DEBUG_THROTTLE(30,"New goal accepted by planner, now plan");

                    ros::WallTime t1 = ros::WallTime::now();

                    planning_ros_msgs::VoxelMap map;
                    map.resolution = map_res;
                    map.origin.x = min_arena_limits[0]/map_res;
                    map.origin.y = min_arena_limits[1]/map_res;
                    map.origin.z = min_arena_limits[2]/map_res;
                    map.dim.x = max_arena_limits[0]/map_res;
                    map.dim.y = max_arena_limits[1]/map_res;
                    map.dim.z = max_arena_limits[2]/map_res;
                    std::vector<int8_t> data(map.dim.x * map.dim.y * map.dim.z, 0);
                    map.data = data;

                    std::array<decimal_t, 3> voxel_map_origin =
                        {map.origin.x, map.origin.y, map.origin.z};

                    vec_Vec3f pts;

                    for (auto& obstacle : last_msg->obstacles) {
                        // Map each obstacle to the cloud
                        float pipe_radius = obstacle.pipe_radius + buffer;
                        float pipe_height = obstacle.pipe_height;
                        float pipe_x = (obstacle.odom.pose.pose.position.x + x_offset);
                        float pipe_y = (obstacle.odom.pose.pose.position.y + y_offset);
                        float px, py, pz;
                        for (pz = voxel_map_origin[2]; pz <= pipe_height; pz += 0.1) {
                            for (float theta = 0; theta < 2 * M_PI; theta += 0.15) {
                                for (float r = 0; r < pipe_radius; r += map_res) {
                                    px = r * std::cos(theta) + pipe_x + voxel_map_origin[0];
                                    py = r * std::sin(theta) + pipe_y + voxel_map_origin[1];
                                    Vec3f pt(px, py, pz);
                                    pts.push_back(pt);
                                }
                            }
                        }
                    }

                    std::array<decimal_t, 3> voxel_map_dim =
                        {max_arena_limits[0],
                         max_arena_limits[1],
                         max_arena_limits[2]};

                    std::unique_ptr<VoxelGrid> voxel_grid(
                        new VoxelGrid(voxel_map_origin, voxel_map_dim, map_res));

                    voxel_grid->addCloud(pts);
                    voxel_grid->getMap(map);

                    // Initialize map util
                    Vec3f ori(map.origin.x, map.origin.y, map.origin.z);
                    Vec3i dim(map.dim.x, map.dim.y, map.dim.z);

                    // map util
                    std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);

                    map_util->setMap(ori, dim, map.data, map_res);

                    ROS_DEBUG_THROTTLE(30,"Takes %f sec for building and setting map",
                        (ros::WallTime::now() - t1).toSec());

                    iarc7_msgs::PlanResult result_;

                    Waypoint3 start;
                    start.pos = Vec3f(pose_start.x, pose_start.y, pose_start.z);
                    start.vel = Vec3f(twist_start.x, twist_start.y, twist_start.z);
                    start.acc = Vec3f(accel_start.x, accel_start.y, accel_start.z);
                    start.use_pos = use_pos;
                    start.use_vel = use_vel;
                    start.use_acc = use_acc;
                    start.use_jrk = use_jrk;

                    Waypoint3 goal;
                    goal.pos = Vec3f(pose_goal.x, pose_goal.y, pose_goal.z);
                    goal.vel = Vec3f(twist_goal.x, twist_goal.y, twist_goal.z);
                    goal.acc = Vec3f(accel_goal.x, accel_goal.y, accel_goal.z);
                    goal.use_pos = use_pos;
                    goal.use_vel = use_vel;
                    goal.use_acc = use_acc;
                    goal.use_jrk = use_jrk;

                    bool done_planning = false;
                    bool valid_plan = false;

                    Trajectory<3> traj;

                    ros::Time time_start = ros::Time::now();
                    ros::Time time_done = time_start + ros::Duration(max_planning_time);

                    int i = 0;
                    double dt = max_dt-i*.05;

                    while (dt >= min_dt && ros::ok() && !done_planning) {
                        std::unique_ptr<MPMap3DUtil> planner;
                        planner.reset(new MPMap3DUtil(false));
                        planner->setMapUtil(map_util);
                        planner->setEpsilon(eps);
                        planner->setVmax(kinematic_constraints[0]);
                        planner->setAmax(kinematic_constraints[1]);
                        planner->setUmax(kinematic_constraints[2]);
                        planner->setDt(dt);
                        planner->setTmax(ndt*dt);
                        planner->setMaxNum(max_num);
                        planner->setU(U);
                        planner->setTol(pose_tol, vel_tol, accel_tol);

                        // now we can plan
                        ros::Time t0 = ros::Time::now();
                        bool valid = planner->plan(start, goal);
                        ros::Duration planning_time = (ros::Time::now() - t0);

                        if (valid) {
                            traj = planner->getTraj();
                            valid_plan = true;
                            done_planning = ((ros::Time::now() + planning_time) < time_done);
                        } else {
                            done_planning = true;
                        }

                        i++;
                        dt = max_dt-i*.05;
                    }

                    if (!valid_plan) {
                        ROS_ERROR("Planner failed to find any plan!");
                        result_.success = valid_plan;
                        server.setSucceeded(result_);
                    } else {
                        // convert trajectory to motion points
                        double traj_time = traj.getTotalTime();
                        double n_samples = samples_per_sec * traj_time;

                        vec_E<Waypoint<3>> waypoints = traj.sample(n_samples);

                        ros::Time time_point = goal_->header.stamp;
                        ros::Duration t_step = ros::Duration(traj_time/waypoints.size());

                        for (Waypoint<3> waypoint : waypoints) {
                            iarc7_msgs::MotionPointStamped m_point;
                            m_point.header.frame_id = "map";
                            m_point.header.stamp = time_point;

                            m_point.motion_point.pose.position.x = waypoint.pos(0)-x_offset;
                            m_point.motion_point.pose.position.y = waypoint.pos(1)-y_offset;
                            m_point.motion_point.pose.position.z = waypoint.pos(2);

                            m_point.motion_point.twist.linear.x = waypoint.vel(0);
                            m_point.motion_point.twist.linear.y = waypoint.vel(1);
                            m_point.motion_point.twist.linear.z = waypoint.vel(2);

                            m_point.motion_point.accel.linear.x = waypoint.acc(0);
                            m_point.motion_point.accel.linear.y = waypoint.acc(1);
                            m_point.motion_point.accel.linear.z = waypoint.acc(2);

                            result_.plan.motion_points.push_back(m_point);
                            time_point = time_point + t_step;
                        }
                        result_.success = valid_plan;
                        result_.total_time = traj_time;
                        server.setSucceeded(result_);
                    }
                }
            } else {
                ROS_DEBUG_THROTTLE(30, "Planner: No new goal.");
            }
        }
        // Handle all ROS callbacks
        ros::spinOnce();
        rate.sleep();
    }
    // All is good.
    return 0;
}
