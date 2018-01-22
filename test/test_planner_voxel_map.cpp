#include "bag_reader.hpp"
#include <ros/ros.h>
#include <ros_utils/data_ros_utils.h>
#include <ros_utils/primitive_ros_utils.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <mapping_utils/voxel_grid.h>
#include <primitive/poly_solver.h>
#include <planner/mp_map_util.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/data_type.h>
#include <iarc7_msgs/ObstacleArray.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>

using namespace MPL;

std::unique_ptr<MPMapUtil> planner_;
std::unique_ptr<VoxelGrid> voxel_grid_;
sensor_msgs::PointCloud map;
planning_ros_msgs::VoxelMap voxel_map;
double start_x, start_y, start_z;
double start_vx, start_vy, start_vz;
double start_ax, start_ay, start_az;
bool odom_set = false;
bool map_set = false;
bool accel_set = false;

float test_origin_x = 10.0;
float test_origin_y = 10.0;
float test_origin_z = 0.0;

ros::Subscriber obst_sub;
ros::Subscriber odom_sub;
ros::Subscriber accel_sub;
ros::Publisher ps_pub;

void obstaclesCallback(const iarc7_msgs::ObstacleArray::ConstPtr& msg)
{
  // Generate map from obstacle data
  ROS_INFO("Number of obstacles: [%d]", msg->obstacles.size());
  map.header.stamp = ros::Time::now();
  map.header.frame_id = "map";
  map.channels.resize(1);

  ROS_INFO("Mapping obstacles to the cloud");
  for (auto &obstacle : msg->obstacles) {
    // Map each obstacle to the cloud

    float pipe_radius = obstacle.pipe_radius;
    float pipe_height = obstacle.pipe_height;
    float pipe_x = obstacle.odom.pose.pose.position.x;
    float pipe_y = obstacle.odom.pose.pose.position.y;
    float px, py, pz;
    for (pz = test_origin_z; pz <= pipe_height; pz += 0.005) {
      for (float theta = 0; theta < 2*M_PI; theta += 0.05){
        px = pipe_radius*cos(theta) + pipe_x + test_origin_x;
        py = pipe_radius*sin(theta) + pipe_y + test_origin_y;
        geometry_msgs::Point32 point;
        point.x = px;
        point.y = py;
        point.z = pz;
        map.points.push_back(point);
        //map.channels[0].values.push_back(100);
      }
    }
  }

  // Add a wall around the arena
  ROS_INFO("Generating wall");
  for (float z = test_origin_z; z <= 5; z += 0.05) {
    for (float i = 0; i <= 20; i += 0.05) {
      geometry_msgs::Point32 point1;
      geometry_msgs::Point32 point2;
      geometry_msgs::Point32 point3;
      geometry_msgs::Point32 point4;
      point1.z = point2.z = point3.z = point4.z = z;
      point1.x = test_origin_x - 10; point1.y = test_origin_y - 10 + i;
      point2.x = test_origin_x - 10 + i; point2.y = test_origin_y + 10;
      point3.x = test_origin_x + 10; point3.y = test_origin_y + 10 - i;
      point4.x = test_origin_x + 10 - i; point4.y = test_origin_y - 10;
      map.points.push_back(point1);
      //map.channels[0].values.push_back(100);
      map.points.push_back(point2);
      //map.channels[0].values.push_back(100);
      map.points.push_back(point3);
      //map.channels[0].values.push_back(100);
      map.points.push_back(point4);
      //map.channels[0].values.push_back(100);
    }
  }
  ROS_INFO("Finished setting map.");
  obst_sub.shutdown();
  map_set = true;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  ROS_INFO("Setting pose and velocity.");
  start_vx = msg->twist.twist.linear.x;
  start_vy = msg->twist.twist.linear.y;
  start_vz = msg->twist.twist.linear.z;

  start_x = msg->pose.pose.position.x + test_origin_x;
  start_y = msg->pose.pose.position.y + test_origin_y;
  start_z = msg->pose.pose.position.z + test_origin_z;

  ROS_INFO("Finished setting odometry.");
  odom_sub.shutdown();
  odom_set = true;
}

void accelCallback(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& msg) {
  ROS_INFO("Setting acceleration.");
  start_ax = msg->accel.accel.linear.x;
  start_ay = msg->accel.accel.linear.y;
  start_az = msg->accel.accel.linear.z;

  accel_set = true;
  accel_sub.shutdown();
  ROS_INFO("Finished setting acceleration.");
}

// Get voxel_map from point cloud
void processCloud() {
  vec_Vec3f pts = cloud_to_vec(map);
  voxel_grid_->addCloud(pts);
  voxel_map = voxel_grid_->getMap();
  voxel_map.header = map.header;
}

// Set voxel map
void setMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util, const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

// Get voxel map
void getMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util, planning_ros_msgs::VoxelMap& map) {
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

bool solve(const Waypoint& start, const Waypoint& goal) {
  ros::Time t0 = ros::Time::now();
  bool valid = planner_->plan(start, goal);

  //Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(planner_->getCloseSet());
  ps.header.frame_id = "map";
  ps_pub.publish(ps);

  if(!valid) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());
    return false;
  }
  else {
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes", (ros::Time::now() - t0).toSec(), planner_->getCloseSet().size());

    return true;
  }
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");
  ros::NodeHandle sub_handle;

  ros::Publisher map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::Polyhedra>("polyhedra", 1, true);
  ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::Ellipsoids>("ellipsoids", 1, true);
  ros::Publisher sg_pub = nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  ros::Publisher traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  ros::Publisher prior_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("prior_trajectory", 1, true);
  ros::Publisher ref_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("trajectory_refined", 1, true);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ps_pub = nh.advertise<sensor_msgs::PointCloud>("ps", 1, true);

  obst_sub = sub_handle.subscribe("obstacles", 1, obstaclesCallback);
  odom_sub = sub_handle.subscribe("odometry/filtered", 1, odometryCallback);
  accel_sub = sub_handle.subscribe("accel/filtered", 1, accelCallback);

  ros::Time t0 = ros::Time::now();

  // Get map, pose, velocity, and acceleration
  ros::Rate loop_rate(10);
  while (ros::ok() && !(map_set && odom_set && accel_set)) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Finished setting required variables.");

  std::string file_name, topic_name;
  nh.param("topic", topic_name, std::string("voxel_map"));
  cloud_pub.publish(map);

  double robot_radius;
  nh.param("robot_r", robot_radius, 0.5);
  Vec3f origin, dim;
  nh.param("origin_x", origin(0), 0.0);
  nh.param("origin_y", origin(1), 0.0);
  nh.param("origin_z", origin(2), 0.0);
  nh.param("range_x", dim(0), 0.0);
  nh.param("range_y", dim(1), 0.0);
  nh.param("range_z", dim(2), 0.0);

  float res = 0.2;

  voxel_grid_.reset(new VoxelGrid(origin, dim, res));

  //Initialize map util
  std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);
  processCloud();
  setMap(map_util, voxel_map);

  //Free unknown space and dilate obstacles
  map_util->freeUnKnown();
  map_util->dilate(0.2, 0.1);
  map_util->dilating();


  //Publish the dilated map for visualization
  getMap(map_util, voxel_map);
  map_pub.publish(voxel_map);
  ROS_INFO("Publish the voxel map! [%zu]", map.points.size());


  ROS_INFO("Takse %f sec to set up map!", (ros::Time::now() - t0).toSec());
  t0 = ros::Time::now();

  //Initialize planner
  double dt, v_max, a_max, w, epsilon, t_max;
  double u_max_z, u_max;
  int max_num, num;
  bool use_3d;
  nh.param("dt", dt, 1.0);
  nh.param("epsilon", epsilon, 1.0);
  nh.param("v_max", v_max, -1.0);
  nh.param("a_max", a_max, -1.0);
  nh.param("u_max", u_max, 1.0);
  nh.param("u_max_z", u_max_z, 1.0);
  nh.param("t_max", t_max, -1.0);
  nh.param("w", w, 10.);
  nh.param("num", num, 1);
  nh.param("max_num", max_num, -1);
  nh.param("use_3d", use_3d, false);

  planner_.reset(new MPMapUtil(true));
  planner_->setMapUtil(map_util); // Set collision checking function
  planner_->setEpsilon(1.0); // Set greedy param (default equal to 1)
  planner_->setVmax(v_max); // Set max velocity
  planner_->setAmax(a_max); // Set max acceleration (as control input)
  planner_->setUmax(a_max);// 2D discretization with 1
  planner_->setDt(dt); // Set dt for each primitive
  planner_->setMaxNum(max_num); // Set maximum allowed expansion, -1 means no limitation
  planner_->setU(1, false);// 2D discretization with 1
  planner_->setTol(1, 1, 1); // Tolerance for goal region

  double goal_x, goal_y, goal_z;
  nh.param("goal_x", goal_x, 6.4);
  nh.param("goal_y", goal_y, 16.6);
  nh.param("goal_z", goal_z, 0.0);

  bool use_acc, use_jrk;
  nh.param("use_acc", use_acc, true);
  nh.param("use_jrk", use_jrk, false);

  // Get Current position, velocity
  Waypoint start;
  start.pos = Vec3f(start_x, start_y, start_z);
  start.vel = Vec3f(start_vx, start_vy, start_vz);
  start.acc = Vec3f(start_ax, start_ay, start_az);
  start.jrk = Vec3f(0, 0, 0);
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = use_acc;
  start.use_jrk = use_jrk;

  Waypoint goal;
  goal.pos = Vec3f(goal_x, goal_y, goal_z);
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;


  //Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header.frame_id = "map";
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
  pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
  sg_pub.publish(sg_cloud);

  //Read prior traj
  std::string traj_file_name, traj_topic_name;
  nh.param("traj_file", traj_file_name, std::string(""));
  nh.param("traj_topic", traj_topic_name, std::string(""));
  bool use_prior;
  nh.param("use_prior", use_prior, false);
  if(!traj_file_name.empty()) {
    planning_ros_msgs::Trajectory prior_traj = read_bag<planning_ros_msgs::Trajectory>(traj_file_name, traj_topic_name, 0).back();
    if(!prior_traj.primitives.empty()) {
      prior_traj_pub.publish(prior_traj);
      if(use_prior) {
        planner_->setPriorTrajectory(toTrajectory(prior_traj));
        goal.use_acc = false;
        goal.use_jrk = false;
      }
    }
  }


  //Set input control
  if(use_3d) {
    vec_Vec3f U;
    decimal_t du = u_max / num;
    decimal_t du_z = u_max_z / num;
    for(decimal_t dx = -u_max; dx <= u_max; dx += du )
      for(decimal_t dy = -u_max; dy <= u_max; dy += du )
        for(decimal_t dz = -u_max_z; dz <= u_max_z; dz += du_z ) //here we reduce the z control
          U.push_back(Vec3f(dx, dy, dz));

    planner_->setU(U);// Set discretization with 1 and efforts
  }
  else
    planner_->setU(num, use_3d); // Set discretization with 1 and efforts
  //planner_->setMode(num, use_3d, start); // Set discretization with 1 and efforts

  //Planning thread!
  if(solve(start, goal)) {

    //Publish trajectory
    Trajectory traj = planner_->getTraj();
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header.frame_id = "map";
    traj_pub.publish(traj_msg);

    printf("================== Traj -- total J(1): %f, J(2): %F, J(3): %f, total time: %f\n",
        traj.J(1), traj.J(2), traj.J(3), traj.getTotalTime());

    /*
    vec_Ellipsoid Es = sample_ellipsoids(traj, Vec3f(robot_radius, robot_radius, 0.1), 50);
    decomp_ros_msgs::Ellipsoids es_msg = DecompROS::ellipsoids_to_ros(Es);
    es_msg.header.frame_id = "map";
    es_pub.publish(es_msg);

    max_attitude(traj, 1000);
    */

    //Get intermediate waypoints
    std::vector<Waypoint> waypoints = planner_->getWs();
    //Get time allocation
    std::vector<decimal_t> dts;
    dts.resize(waypoints.size() - 1, dt);

    //Generate higher order polynomials
    PolySolver poly_solver(2,3);
    poly_solver.solve(waypoints, dts);

    Trajectory traj_refined = Trajectory(poly_solver.getTrajectory()->toPrimitives());

    //Publish refined trajectory
    traj_msg = toTrajectoryROSMsg(traj_refined);
    traj_msg.header.frame_id = "map";
    ref_traj_pub.publish(traj_msg);

    printf("================ Refined traj -- total J: %f, total time: %f\n", traj_refined.J(1), traj_refined.getTotalTime());
  }

  /*
  decomp_ros_msgs::Polyhedra poly_msg = DecompROS::polyhedra_to_ros(planner_->getPolyhedra());
  poly_msg.header.frame_id = "map";
  poly_pub.publish(poly_msg);
  */


  ros::spin();

  return 0;
}
