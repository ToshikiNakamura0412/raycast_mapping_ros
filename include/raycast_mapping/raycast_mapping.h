/**
 * @file raycast_mapping.h
 * @author Toshiki Nakamura
 * @brief C++ implementation of Ray Casting Update Algorithm for 2D Mapping
 * @copyright Copyright (c) 2024
 */

#ifndef RAYCAST_MAPPING_RAYCAST_MAPPING_H
#define RAYCAST_MAPPING_RAYCAST_MAPPING_H

#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <utility>
#include <vector>

/**
 * @struct PrecastData
 * @brief Struct for precast data
 */
struct PrecastData
{
  float dist;
  int grid_index;
};

/**
 * @struct PrecastDB
 * @brief Struct for precast database
 */
struct PrecastDB
{
  nav_msgs::MapMetaData info;
  float yaw_reso;
  std::vector<std::vector<PrecastData>> bins;
};

/**
 * @class LocalMapCreator
 * @brief Class for creating local map
 */
class LocalMapCreator
{
public:
  /**
   * @brief Construct a new Local Map Creator object
   */
  LocalMapCreator(void);

private:
  /**
   * @brief Callback function for point cloud
   *
   * @param msg Point cloud message
   */
  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * @brief Create precast database
   *
   * @param map_reso Map resolution
   * @param map_size Map size
   * @param yaw_reso Yaw resolution
   * @return PrecastDB Precast database
   */
  PrecastDB create_precast_db(const float map_reso, const float map_size, const float yaw_reso);

  /**
   * @brief Calculate distance and angle
   *
   * @param index Index
   * @param map_info Map information
   * @return std::pair<float, float> Distance and angle
   */
  std::pair<float, float> calc_dist_and_angle(const int index, const nav_msgs::MapMetaData &map_info);

  /**
   * @brief Calculate angle ID
   *
   * @param angle Angle
   * @param yaw_reso Yaw resolution
   * @return int Angle ID
   */
  int calc_angle_id(float angle, const float yaw_reso);

  /**
   * @brief Create map
   *
   * @param cloud Point cloud
   * @param precast_db Precast database
   * @return nav_msgs::OccupancyGrid Local map
   */
  nav_msgs::OccupancyGrid create_map(const sensor_msgs::PointCloud2 &cloud, const PrecastDB &precast_db);

  /**
   * @brief Initialize map
   *
   * @param map_info Map information
   * @return nav_msgs::OccupancyGrid Initialized map
   */
  nav_msgs::OccupancyGrid init_map(const nav_msgs::MapMetaData &map_info);

  /**
   * @brief Check if the point is in the map
   *
   * @param dist Distance
   * @param angle Angle
   * @param map_info Map information
   * @return true If the point is in the map
   * @return false If the point is not in the map
   */
  bool in_map(const float dist, const float angle, const nav_msgs::MapMetaData &map_info);

  /**
   * @brief Convert x, y to grid index
   *
   * @param x X
   * @param y Y
   * @param map_info Map information
   * @return int Grid index
   */
  int xy_to_grid_index(const float x, const float y, const nav_msgs::MapMetaData &map_info);

  float map_reso_;  // [m/cell]
  float map_size_;
  float yaw_reso_;  // [rad/cell]
  PrecastDB precast_db_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher map_pub_;
  ros::Subscriber cloud_sub_;
};

#endif  // RAYCAST_MAPPING_RAYCAST_MAPPING_H
