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

typedef actionlib::SimpleActionServer<iarc7_msgs::PlanAction> Server;

enum class PlannerState { WAITING, PLANNING };


void generateVoxelMap(planning_ros_msgs::VoxelMap &voxel_map,
                      const iarc7_msgs::ObstacleArray &obstacles) {
    ros::Time t1 = ros::Time::now();

    sensor_msgs::PointCloud cloud;

    // Generate cloud from obstacle data
    ROS_INFO("Number of obstacles: [%zu]", obstacles.obstacles.size());
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "cloud";
    cloud.channels.resize(1);

    Vec3f voxel_map_origin;
    voxel_map_origin(0) = voxel_map.origin.x;
    voxel_map_origin(1) = voxel_map.origin.y;
    voxel_map_origin(2) = voxel_map.origin.z;

    ROS_INFO("Mapping obstacles to the cloud");
    for (auto &obstacle : obstacles.obstacles) {
        // Map each obstacle to the cloud

        float pipe_radius = obstacle.pipe_radius;
        float pipe_height = obstacle.pipe_height;
        float pipe_x = obstacle.odom.pose.pose.position.x;
        float pipe_y = obstacle.odom.pose.pose.position.y;
        float px, py, pz;
        for (pz = voxel_map_origin(2); pz <= pipe_height; pz += 0.1) {
            for (float theta = 0; theta < 2 * M_PI; theta += 0.15) {
                for (float r = pipe_radius - voxel_map.resolution;
                     r < pipe_radius; r += voxel_map.resolution) {
                    px = r * cos(theta) + pipe_x + voxel_map_origin(0);
                    py = r * sin(theta) + pipe_y + voxel_map_origin(1);
                    geometry_msgs::Point32 point;
                    point.x = px;
                    point.y = py;
                    point.z = pz;
                    cloud.points.push_back(point);
                }
            }
        }
    }

    ROS_INFO("Takes %f sec for mapping obstalces",
             (ros::Time::now() - t1).toSec());

    t1 = ros::Time::now();

    vec_Vec3f pts = cloud_to_vec(cloud);

    ROS_INFO("Takes %f sec for cloud to vec", (ros::Time::now() - t1).toSec());

    Vec3f voxel_map_dim;
    voxel_map_dim(0) = voxel_map.dim.x * voxel_map.resolution;
    voxel_map_dim(1) = voxel_map.dim.y * voxel_map.resolution;
    voxel_map_dim(2) = voxel_map.dim.z * voxel_map.resolution;

    std::unique_ptr<VoxelGrid> voxel_grid(
        new VoxelGrid(voxel_map_origin, voxel_map_dim, voxel_map.resolution));
    voxel_grid->addCloud(pts);

    voxel_map = voxel_grid->getMap();
    voxel_map.header = cloud.header;
}

void setMap(std::shared_ptr<MPL::VoxelMapUtil> &map_util,
            const planning_ros_msgs::VoxelMap &msg) {
    Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
    Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
    decimal_t res = msg.resolution;
    std::vector<signed char> map = msg.data;

    map_util->setMap(ori, dim, map, res);
}

void getMap(std::shared_ptr<MPL::VoxelMapUtil> &map_util,
            planning_ros_msgs::VoxelMap &map) {
    Vec3f ori = map_util->getOrigin();
    Vec3i dim = map_util->getDim();
    decimal_t res = map_util->getRes();

    map.origin.x = ori(0);
    map.origin.y = ori(1);
    map.origin.z = ori(2);

    map.dim.x = dim(0);
    map.dim.y = dim(1);
    map.dim.z = dim(2);
    map.resolution = res;

    map.data = map_util->getMap();
}





bool checkGoal(Pose &pose_goal, Twist &twist_goal, Accel &accel_goal,
               double kinematic_constraints[3], double max_arena[3],
               double min_arena[3]) {
    if (pose_goal.position.x > max_arena[0] ||
        pose_goal.position.y > max_arena[1] ||
        pose_goal.position.z > max_arena[2]) {
        ROS_ERROR("Goal provided to planner is above arena limits");
        return false;
    }

    if (pose_goal.position.x < min_arena[0] ||
        pose_goal.position.z < min_arena[1] ||
        pose_goal.position.z < min_arena[2]) {
        ROS_ERROR("Goal provided to planner is below arena limits");
        return false;
    }

    if (std::max({std::abs(twist_goal.linear.x), std::abs(twist_goal.linear.y),
                  std::abs(twist_goal.linear.z)}) > kinematic_constraints[0]) {
        ROS_ERROR(
            "Goal provided to planner is beyond platform velocity limits");
        return false;
    }

    if (std::max({std::abs(accel_goal.linear.x), std::abs(accel_goal.linear.y),
                  std::abs(accel_goal.linear.z)}) > kinematic_constraints[1]) {
        ROS_ERROR("Goal provided to planner is beyond platform acceleration limits");
        return false;
    }

    return true;
}

// Main entry point for the motion planner
int main(int argc, char **argv) {
    // Required by ROS before calling many functions
    ros::init(argc, argv, "Motion_Planner");

    ROS_INFO("Motion_Planner begin");

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

    // LOAD PARAMETERS

    // node update frequency
    double update_frequency = 20;

    // Update frequency retrieve
    ROS_ASSERT(private_nh.getParam("planner_update_frequency", update_frequency));

    // settings for planner

    // min and max arena limits
    double max_arena_limits[3] = {20, 20, 20};
    double min_arena_limits[3] = {0, 0, 0};

    // speed, acceleration, jerk limits
    double kinematic_constraints[3] = {3, 5, 15};

    // pos, vel, accel tolerances
    double tolerances[3] = {.2, .2, .5};

    double dt = .26;
    double eps = 10;
    double u_max = 20;

    // resolution to generate map at
    double map_res = .25;

    int max_num = 5000;
    int num = 1;
    int ndt = 1000;

    bool use_pos = true;
    bool use_vel = true;
    bool use_acc = true;
    bool use_jrk = false;

    vec_Vec3f U;

    // max x, y, z arena limits
    ROS_ASSERT(private_nh.getParam("arena/max_x", max_arena_limits[0]));
    ROS_ASSERT(private_nh.getParam("arena/max_y", max_arena_limits[1]));
    ROS_ASSERT(private_nh.getParam("arena/max_z", max_arena_limits[2]));

    // min x, y, z arena limits
    ROS_ASSERT(private_nh.getParam("arena/min_x", min_arena_limits[0]));
    ROS_ASSERT(private_nh.getParam("arena/min_y", min_arena_limits[1]));
    ROS_ASSERT(private_nh.getParam("arena/min_z", min_arena_limits[2]));

    // resolution to generate map at
    ROS_ASSERT(private_nh.getParam("map_res", map_res));

    // speed, acceleration, jerk limits
    ROS_ASSERT(private_nh.getParam("max_speed", kinematic_constraints[0]));
    ROS_ASSERT(private_nh.getParam("max_acceleration", kinematic_constraints[1]));
    ROS_ASSERT(private_nh.getParam("max_jerk", kinematic_constraints[2]));

    // time discretization for each primitive
    ROS_ASSERT(private_nh.getParam("dt", dt));

    ROS_ASSERT(private_nh.getParam("use_pos", use_pos));
    ROS_ASSERT(private_nh.getParam("use_vel", use_vel));
    ROS_ASSERT(private_nh.getParam("use_acc", use_acc));
    ROS_ASSERT(private_nh.getParam("use_jrk", use_jrk));

    // ndt*dt is the plannign horizon
    ROS_ASSERT(private_nh.getParam("ndt", ndt));

    // Epsilon: greedy param
    ROS_ASSERT(private_nh.getParam("eps", eps));

    // control discretization
    ROS_ASSERT(private_nh.getParam("num", num));

    // max for control space
    ROS_ASSERT(private_nh.getParam("u_max", u_max));

    // maximum number of allowed expansion
    ROS_ASSERT(private_nh.getParam("max_num", max_num));

    // kinematic tolerances
    ROS_ASSERT(private_nh.getParam("p_tol", tolerances[0]));
    ROS_ASSERT(private_nh.getParam("v_tol", tolerances[1]));
    ROS_ASSERT(private_nh.getParam("a_tol", tolerances[2]));

    const decimal_t du = u_max / num;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du) {
        for (decimal_t dy = -u_max; dy <= u_max; dy += du) {
            for (decimal_t dz = -u_max; dz <= u_max; dz += du) {
                U.push_back(Vec3f(dx, dy, dz));
            }
        }
    }

    // debug publishers
    ros::Publisher map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
    ros::Publisher traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);

    std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);


    planning_ros_msgs::VoxelMap map;

    ros::Time start_map = ros::Time::now();

    map.resolution = map_res;

    map.origin.x = min_arena_limits[0];
    map.origin.y = min_arena_limits[1];
    map.origin.z = min_arena_limits[2];

    map.dim.x = max_arena_limits[0];
    map.dim.y = max_arena_limits[1];
    map.dim.z = max_arena_limits[2];

    std::vector<int8_t> data(map.dim.x*map.dim.y*map.dim.z, 0);
    map.data = data;

    // Initialize map util
    setMap(map_util, map);

    // initialize planner
    std::unique_ptr<MPMap3DUtil> planner;
    planner.reset(new MPMap3DUtil(true));
    planner->setMapUtil(map_util);
    planner->setEpsilon(eps);
    planner->setVmax(kinematic_constraints[0]);
    planner->setAmax(kinematic_constraints[1]);
    planner->setUmax(u_max);
    planner->setDt(dt);
    planner->setTmax(ndt*dt);
    planner->setMaxNum(max_num);
    planner->setU(U);
    planner->setTol(tolerances[0]);

    iarc7_msgs::PlanGoalConstPtr goal_;

    Pose pose_goal;
    Twist twist_goal;
    Accel accel_goal;

    // Form a connection with the node monitor. If no connection can be made
    // assert because we don't know what's going on with the other nodes.
    // ROS_INFO("Motion_Planner: Attempting to form safety bond");

    // Iarc7Safety::SafetyClient safety_client(nh, "motion_planner");

    // ROS_ASSERT_MSG(safety_client.formBond(),"Motion_Planner: Could not form bond with safety client");

    // Cache the time
    ros::Time last_time = ros::Time::now();

    ros::Rate rate(update_frequency);

    PlannerState state = PlannerState::WAITING;

    // Run until ROS says we need to shutdown
    while (ros::ok()) {
        // Check the safety client before updating anything
        // there is no safety response for this node, so
        // shut down.

        // ROS_ASSERT_MSG(!safety_client.isFatalActive(), "motion_planner: fatal event from safety");
        // ROS_ASSERT_MSG(!safety_client.isSafetyActive(), "Motion_Planner shutdown due to safety active");

        // Get the time
        ros::Time current_time = ros::Time::now();

        // Make sure we have a new timestamp. This can be a problem with
        // simulated time that does not update with high precision.
        if (current_time > last_time) {
            last_time = current_time;

            // goal was canceled
            if (server.isPreemptRequested() &&
                state == PlannerState::PLANNING) {
                server.setPreempted();
                ROS_INFO(
                    "Preempt requested. Current planning goal was canceled");
                state = PlannerState::WAITING;
            }

            // get a new goal from action server
            if (server.isNewGoalAvailable()) {
                if (server.isActive()) {
                    ROS_INFO(
                        "New goal received with current goal still running");
                }

                // planning goal
                goal_ = server.acceptNewGoal();

                pose_goal = goal_->goal.motion_point.pose;
                twist_goal = goal_->goal.motion_point.twist;
                accel_goal = goal_->goal.motion_point.accel;

                if (!checkGoal(pose_goal, twist_goal, accel_goal,
                               kinematic_constraints,
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

                ros::Time start_map = ros::Time::now();

                map.resolution = map_res;

                map.origin.x = min_arena_limits[0];
                map.origin.y = min_arena_limits[1];
                map.origin.z = min_arena_limits[2];

                map.dim.x = max_arena_limits[0];
                map.dim.y = max_arena_limits[1];
                map.dim.z = max_arena_limits[2];

                std::vector<int8_t> data(map.dim.x*map.dim.y*map.dim.z, 0);
                map.data = data;

                iarc7_msgs::ObstacleArray obstacles;

                iarc7_msgs::Obstacle new_obstacle;
                new_obstacle.pipe_height = 2.0;
                new_obstacle.pipe_radius = 1.0;
                new_obstacle.odom.pose.pose.position.x = 9;
                new_obstacle.odom.pose.pose.position.y = 11;
                new_obstacle.odom.pose.pose.position.z = 0;
                obstacles.obstacles.push_back(new_obstacle);

                for (double i = 0; i < 2; i += 0.8) {
                    new_obstacle.pipe_height = 2.0;
                    new_obstacle.pipe_radius = 1.0;
                    new_obstacle.odom.pose.pose.position.x = 15 - i * 2;
                    new_obstacle.odom.pose.pose.position.y = 15 + 0.5 * i;
                    new_obstacle.odom.pose.pose.position.z = 0;
                    obstacles.obstacles.push_back(new_obstacle);
                }

                generateVoxelMap(map, obstacles);

                // Initialize map util
                setMap(map_util, map);

                // Free unknown space and dilate obstacles
                map_util->freeUnknown();
                // map_util->dilate(0.2, 0.1);

                ROS_INFO("Takes %f sec for building map", (ros::Time::now() - start_map).toSec());

                // Publish the dilated map for visualization
                getMap(map_util, map);
                map.header.frame_id = "map";
                map_pub.publish(map);

                // Set start and goal
                double start_x = 1.0;
                double start_y = 1.0;
                double start_z = 0.5;

                double start_vx = 0.0;
                double start_vy = 0.0;
                double start_vz = 0.1;

                Waypoint3 start;
                start.pos = Vec3f(start_x, start_y, start_z);
                start.vel = Vec3f(start_vx, start_vy, start_vz);
                start.acc = Vec3f(0, 0, 0);
                start.use_pos = use_pos;
                start.use_vel = use_vel;
                start.use_acc = use_acc;
                start.use_jrk = use_jrk;

                Waypoint3 goal;
                goal.pos = Vec3f(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z);
                goal.vel = Vec3f(twist_goal.linear.x, twist_goal.linear.y, twist_goal.linear.z);
                goal.acc = Vec3f(accel_goal.linear.x, accel_goal.linear.y, accel_goal.linear.z);
                goal.use_pos = use_pos;
                goal.use_vel = use_vel;
                goal.use_acc = use_acc;
                goal.use_jrk = use_jrk;

                planner->setMapUtil(map_util);
                planner->setU(U);
                planner->setUmax(u_max);

                // now we can plan
                ros::Time t0 = ros::Time::now();
                bool valid = planner->plan(start, goal);

                if (!valid) {
                    ROS_WARN("Failed and took %f sec for planning, expand [%zu] nodes",
                        (ros::Time::now() - t0).toSec(), planner->getCloseSet().size());
                } else {
                    ROS_INFO("Succeeded and took %f sec for planning, expand [%zu] nodes",
                        (ros::Time::now() - t0).toSec(), planner->getCloseSet().size());
                }

                // Publish trajectory
                auto traj = planner->getTraj();
                planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
                traj_msg.header.frame_id = "map";
                traj_pub.publish(traj_msg);

                result_.success = valid;

                server.publishFeedback(feedback_);

                state = PlannerState::WAITING;
            }
        }

        // Handle all ROS callbacks
        ros::spinOnce();
        rate.sleep();
    }
    // All is good.
    return 0;
}
