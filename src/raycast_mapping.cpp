/**
 * @file raycast_mapping.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of Ray Casting Update Algorithm for 2D Mapping
 * @copyright Copyright (c) 2024
 */

#include <utility>
#include <vector>

#include "raycast_mapping/raycast_mapping.h"

LocalMapCreator::LocalMapCreator(void) : private_nh_("~")
{
  private_nh_.param<float>("map_reso", map_reso_, 0.05);
  private_nh_.param<float>("map_size", map_size_, 10.0);
  private_nh_.param<float>("yaw_reso", yaw_reso_, 0.087);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
  cloud_sub_ = nh_.subscribe("/cloud", 1, &LocalMapCreator::cloud_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  ROS_INFO_STREAM("map_reso: " << map_reso_);
  ROS_INFO_STREAM("map_size: " << map_size_);
  ROS_INFO_STREAM("yaw_reso: " << yaw_reso_);

  precast_db_ = create_precast_db(map_reso_, map_size_, yaw_reso_);
}

void LocalMapCreator::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  nav_msgs::OccupancyGrid local_map = create_map(*msg, precast_db_);
  local_map.header.frame_id = msg->header.frame_id;
  local_map.header.stamp = ros::Time::now();
  map_pub_.publish(local_map);
}

PrecastDB LocalMapCreator::create_precast_db(const float map_reso, const float map_size, const float yaw_reso)
{
  PrecastDB precast_db;
  precast_db.info.resolution = map_reso;
  precast_db.info.width = static_cast<int>(round(map_size / map_reso));
  precast_db.info.height = static_cast<int>(round(map_size / map_reso));
  precast_db.info.origin.position.x = -map_size / 2.0;
  precast_db.info.origin.position.y = -map_size / 2.0;
  precast_db.yaw_reso = yaw_reso;
  precast_db.bins.resize(2.0 * M_PI / yaw_reso + 1);

  for (int i = 0; i < precast_db.info.width * precast_db.info.height; i++)
  {
    const std::pair<float, float> dist_and_angle = calc_dist_and_angle(i, precast_db.info);
    const int angle_id = calc_angle_id(dist_and_angle.second, yaw_reso);

    PrecastData data;
    data.dist = dist_and_angle.first;
    data.grid_index = i;
    try
    {
      precast_db.bins[angle_id].push_back(data);
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM(e.what());
      exit(1);
    }
  }

  ROS_WARN_STREAM("Precast DB has been created");
  return precast_db;
}

std::pair<float, float> LocalMapCreator::calc_dist_and_angle(const int index, const nav_msgs::MapMetaData &map_info)
{
  const int index_x = index % map_info.width;
  const int index_y = static_cast<int>(index / map_info.width);
  const float x = index_x * map_info.resolution + map_info.origin.position.x;
  const float y = index_y * map_info.resolution + map_info.origin.position.y;
  return std::make_pair(hypot(x, y), atan2(y, x));
}

int LocalMapCreator::calc_angle_id(float angle, const float yaw_reso)
{
  angle = angle < 0 ? angle + 2.0 * M_PI : angle;
  return static_cast<int>(floor(angle / yaw_reso));
}

nav_msgs::OccupancyGrid LocalMapCreator::create_map(const sensor_msgs::PointCloud2 &cloud, const PrecastDB &precast_db)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(cloud, pcl_cloud);
  nav_msgs::OccupancyGrid local_map = init_map(precast_db.info);

  for (const auto &point : pcl_cloud.points)
  {
    if (!in_map(point.x, point.y, precast_db.info))
      continue;

    const float dist = hypot(point.x, point.y);
    const float angle = atan2(point.y, point.x);
    const int angle_id = calc_angle_id(angle, precast_db.yaw_reso);

    const std::vector<PrecastData> &precast_data = precast_db.bins[angle_id];
    for (const auto &data : precast_data)
    {
      if (dist < data.dist)
        local_map.data[data.grid_index] = -1;  // unknown
    }
    local_map.data[xy_to_grid_index(point.x, point.y, precast_db.info)] = 100;  // occupied
  }

  return local_map;
}

nav_msgs::OccupancyGrid LocalMapCreator::init_map(const nav_msgs::MapMetaData &map_info)
{
  nav_msgs::OccupancyGrid local_map;
  local_map.info = map_info;
  local_map.data.reserve(map_info.width * map_info.height);
  for (int i = 0; i < map_info.width * map_info.height; i++)
    local_map.data.push_back(0);  // free

  return local_map;
}

bool LocalMapCreator::in_map(const float x, const float y, const nav_msgs::MapMetaData &map_info)
{
  return map_info.origin.position.x <= x && x <= map_info.origin.position.x + map_info.width * map_info.resolution &&
         map_info.origin.position.y <= y && y <= map_info.origin.position.y + map_info.height * map_info.resolution;
}

int LocalMapCreator::xy_to_grid_index(const float x, const float y, const nav_msgs::MapMetaData &map_info)
{
  const int index_x = static_cast<int>(round((x - map_info.origin.position.x) / map_info.resolution));
  const int index_y = static_cast<int>(round((y - map_info.origin.position.y) / map_info.resolution));
  return index_x + (index_y * map_info.width);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "raycast_mapping");
  LocalMapCreator local_map_creator;
  ros::spin();

  return 0;
}
