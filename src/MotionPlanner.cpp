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

enum class PlannerState { WAITING, PLANNING };


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

    double samples_per_sec = 50;

    // samples per sec for generating waypoints
    double obstacle_coordinate_offset = 10;

    int max_num = 5000;
    int num = 1;
    int ndt = 1000;

    bool use_pos = true;
    bool use_vel = true;
    bool use_acc = true;
    bool use_jrk = false;

    vec_Vec3f U;

    // Update frequency retrieve
    ROS_ASSERT(private_nh.getParam("planner_update_frequency", update_frequency));

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

    // ndt*dt is the planning horizon
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

    // correction for obstacle-planner coordinate frame differences
    ROS_ASSERT(private_nh.getParam("obstacle_coordinate_offset", obstacle_coordinate_offset));

    ROS_ASSERT(private_nh.getParam("samples_per_sec", samples_per_sec));

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

    // for caching maps
    bool new_obstacle_available = false;
    planning_ros_msgs::VoxelMap last_map;

    // map util
    std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);

    // debug publishers
    ros::Publisher map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
    ros::Publisher traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);

    // subscribe to obstacle topic

    boost::shared_ptr<const iarc7_msgs::ObstacleArray> last_msg;
    boost::function<void(const boost::shared_ptr<const iarc7_msgs::ObstacleArray>&)> obstacle_callback =
        [&](const boost::shared_ptr<const iarc7_msgs::ObstacleArray>& msg) -> void {
            if (last_msg == nullptr || last_msg->header.stamp < msg->header.stamp) {
                last_msg = msg;
                new_obstacle_available = true;

            } else {
                ROS_ERROR("Bad stamp on obstacle message");
            }
        };
    ros::Subscriber obstacle_sub = nh.subscribe("obstacles", 2, obstacle_callback);

    // Form a connection with the node monitor. If no connection can be made
    // assert because we don't know what's going on with the other nodes.
    ROS_INFO("Motion_Planner: Attempting to form safety bond");

    Iarc7Safety::SafetyClient safety_client(nh, "motion_planner");

    ROS_ASSERT_MSG(safety_client.formBond(),"Motion_Planner: Could not form bond with safety client");

    // Cache the time
    ros::Time last_time = ros::Time::now();

    // node update rate
    ros::Rate rate(update_frequency);

    PlannerState state = PlannerState::WAITING;

    // Run until ROS says we need to shutdown
    while (ros::ok()) {
        // Check the safety client before updating anything
        // there is no safety response for this node, so
        // shut down.

        ROS_ASSERT_MSG(!safety_client.isFatalActive(), "motion_planner: fatal event from safety");
        ROS_ASSERT_MSG(!safety_client.isSafetyActive(), "Motion_Planner shutdown due to safety active");

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

            if (last_msg == nullptr) {
                ROS_ERROR("MotionPlanner: No obstacle information available, can't build map");
                new_obstacle_available = false;
                state = PlannerState::WAITING;
            } else if (state == PlannerState::WAITING) {
                // get a new goal from action server
                if (server.isNewGoalAvailable()) {
                    if (server.isActive()) {
                        ROS_INFO("New goal received with current goal still running");
                    }

                    // planning goal
                    goal_ = server.acceptNewGoal();

                    pose_goal = goal_->goal.motion_point.pose.position;
                    twist_goal = goal_->goal.motion_point.twist.linear;
                    accel_goal = goal_->goal.motion_point.accel.linear;

                    pose_start = goal_->start.motion_point.pose.position;
                    twist_start = goal_->start.motion_point.twist.linear;
                    accel_start = goal_->start.motion_point.accel.linear;

                    if (!checkGoal(pose_goal, twist_goal, accel_goal,
                                   kinematic_constraints,
                                   max_arena_limits,
                                   min_arena_limits)) {
                        ROS_ERROR("Planner aborting requested gaol");
                        server.setAborted();
                        state = PlannerState::WAITING;
                    } else {
                        ROS_INFO("New goal accepted by planner, now map");
                    }

                    state = PlannerState::PLANNING;

                } else {
                    ROS_DEBUG_THROTTLE(60, "Planner: No new goal. Generate map only");
                    state = PlannerState::WAITING;
                }

                if (new_obstacle_available) {
                    ros::WallTime t1 = ros::WallTime::now();

                    // Standard header
                    std_msgs::Header header;
                    header.frame_id = std::string("map");

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

                    sensor_msgs::PointCloud cloud;

                    // Generate cloud from obstacle data
                    //ROS_INFO("Number of obstacles: [%zu]", last_msg->obstacles.size());
                    cloud.header.stamp = ros::Time::now();
                    cloud.header.frame_id = "cloud";
                    cloud.channels.resize(1);

                    std::array<decimal_t, 3> voxel_map_origin =
                        {map.origin.x, map.origin.y, map.origin.z};

                    //ROS_INFO("Mapping obstacles to the cloud");
                    for (auto& obstacle : last_msg->obstacles) {
                        // Map each obstacle to the cloud
                        float pipe_radius = obstacle.pipe_radius;
                        float pipe_height = obstacle.pipe_height;
                        float pipe_x = (obstacle.odom.pose.pose.position.x +
                                    obstacle_coordinate_offset)/map_res;
                        float pipe_y = (obstacle.odom.pose.pose.position.y +
                                    obstacle_coordinate_offset)/map_res;
                        float px, py, pz;
                        for (pz = voxel_map_origin[2]; pz <= pipe_height; pz += 0.1) {
                            for (float theta = 0; theta < 2 * M_PI; theta += 0.15) {
                                for (float r = pipe_radius - map_res; r < pipe_radius; r += map_res) {
                                    px = r * std::cos(theta) + pipe_x + voxel_map_origin[0];
                                    py = r * std::sin(theta) + pipe_y + voxel_map_origin[1];
                                    geometry_msgs::Point32 point;
                                    point.x = px;
                                    point.y = py;
                                    point.z = pz;
                                    cloud.points.push_back(point);
                                }
                            }
                        }
                    }

                    vec_Vec3f pts = cloud_to_vec(cloud);

                    std::array<decimal_t, 3> voxel_map_dim =
                        {map.dim.x * map_res,
                         map.dim.y * map_res,
                         map.dim.z * map_res};

                    std::unique_ptr<VoxelGrid> voxel_grid(
                        new VoxelGrid(voxel_map_origin, voxel_map_dim, map_res));

                    voxel_grid->addCloud(pts);
                    voxel_grid->getMap(map);

                    map.header = cloud.header;

                    // Initialize map util
                    Vec3f ori(map.origin.x, map.origin.y, map.origin.z);
                    Vec3i dim(map.dim.x, map.dim.y, map.dim.z);

                    map_util->setMap(ori, dim, map.data, map_res);

                    // Free unknown space and dilate obstacles
                    // map_util->freeUnknown();
                    // map_util->dilate(0.2, 0.1);

                    //ROS_INFO("Takes %f sec for building map", (ros::WallTime::now() - t1).toSec());

                    // Publish the dilated map for visualization
                    map.header = header;
                    map_pub.publish(map);

                    last_map = map;

                    new_obstacle_available = false;
                } else if (last_msg != nullptr) {
                    //ROS_INFO("MotionPlanner: No new obstacle info available. Using previously generated map");

                    Vec3f ori(last_map.origin.x, last_map.origin.y, last_map.origin.z);
                    Vec3i dim(last_map.dim.x, last_map.dim.y, last_map.dim.z);

                    map_util->setMap(ori, dim, last_map.data, map_res);
                }
            }

            // able to generate a plan
            if (state == PlannerState::PLANNING) {
                iarc7_msgs::PlanResult result_;
                iarc7_msgs::PlanFeedback feedback_;

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

                std::unique_ptr<MPMap3DUtil> planner;
                planner.reset(new MPMap3DUtil(true));
                planner->setMapUtil(map_util);
                planner->setEpsilon(eps);
                planner->setVmax(kinematic_constraints[0]);
                planner->setAmax(kinematic_constraints[1]);
                planner->setUmax(kinematic_constraints[2]);
                planner->setDt(dt);
                planner->setTmax(ndt * dt);
                planner->setMaxNum(max_num);
                planner->setU(U);
                planner->setTol(tolerances[0]);

                // now we can plan
                ros::WallTime t0 = ros::WallTime::now();
                bool valid = planner->plan(start, goal);

                if (!valid) {
                    ROS_WARN("Failed and took %f sec for planning", (ros::WallTime::now() - t0).toSec());
                    ROS_ERROR("Planner failed to find a plan, will try again");

                    feedback_.success = valid;
                    server.publishFeedback(feedback_);
                } else {
                    ROS_INFO("Succeeded and took %f sec for planning, expand [%zu] nodes",
                        (ros::WallTime::now() - t0).toSec(), planner->getCloseSet().size());

                    // convert trajectory to motion points
                    auto traj = planner->getTraj();

                    double traj_time = traj.getTotalTime();
                    double n_samples = samples_per_sec * traj_time;

                    vec_E<Waypoint<3>> waypoints = traj.sample(n_samples);

                    ros::Time time_point = ros::Time::now();
                    ros::Duration t_step = ros::Duration(traj_time/waypoints.size());

                    for (Waypoint<3> waypoint : waypoints) {
                        iarc7_msgs::MotionPointStamped m_point;
                        m_point.header.frame_id = "map";
                        m_point.header.stamp = time_point;

                        m_point.motion_point.pose.position.x = waypoint.pos(0);
                        m_point.motion_point.pose.position.y = waypoint.pos(1);
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
                    result_.success = valid;
                    result_.total_time = traj_time;

                    server.setSucceeded(result_);

                    state = PlannerState::WAITING;
                }
            }
        }

        // Handle all ROS callbacks
        ros::spinOnce();
        rate.sleep();
    }
    // All is good.
    return 0;
}
