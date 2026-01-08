#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
    : logger_(logger),
      map_initialized_(false)
{
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::integrateCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap, 
    const nav_msgs::msg::Odometry& odom) 
{
    if (!map_initialized_) {
        initializeGlobalMap(costmap);
        map_initialized_ = true;
    } 
       
    mergeCostmapIntoGlobalMap(costmap, odom);
    
    return global_map_;
}

void MapMemoryCore::initializeGlobalMap(const nav_msgs::msg::OccupancyGrid& first_costmap)
{
    global_map_.info.resolution = first_costmap.info.resolution;

    global_map_.info.width  = 800;
    global_map_.info.height = 800;

    global_map_.info.origin.position.x = -(global_map_.info.width  * global_map_.info.resolution) / 2.0;
    global_map_.info.origin.position.y = -(global_map_.info.height * global_map_.info.resolution) / 2.0;

    global_map_.header.frame_id = "sim_world";

    global_map_.data.assign(
        global_map_.info.width * global_map_.info.height, 0);
}

void MapMemoryCore::mergeCostmapIntoGlobalMap(
    const nav_msgs::msg::OccupancyGrid& costmap,
    const nav_msgs::msg::Odometry& odom)
{
    if (costmap.data.empty())
        return;

    const int cmap_w = costmap.info.width;
    const int cmap_h = costmap.info.height;
    const double cmap_res = costmap.info.resolution;

    const double cx = cmap_w * cmap_res / 2.0;
    const double cy = cmap_h * cmap_res / 2.0;

    const int gmap_w = global_map_.info.width;
    const int gmap_h = global_map_.info.height;
    const double gmap_res = global_map_.info.resolution;

    const double origin_x = costmap.info.origin.position.x;
    const double origin_y = costmap.info.origin.position.y;

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const auto& q = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    for (int y = 0; y < cmap_h; ++y) {
        for (int x = 0; x < cmap_w; ++x) {

            const int cmap_ind = y * cmap_w + x;
            const int8_t val = costmap.data[cmap_ind];

            if (val == -1)
                continue;

            const double lx = x * cmap_res - cx;
            const double ly = y * cmap_res - cy;

            const double rx = std::cos(yaw) * lx - std::sin(yaw) * ly;
            const double ry = std::sin(yaw) * lx + std::cos(yaw) * ly;

            const double wx = origin_x + x0 + rx + cx;
            const double wy = origin_y + y0 + ry + cy;

            const int gx = static_cast<int>((wx - global_map_.info.origin.position.x) / gmap_res);
            const int gy = static_cast<int>((wy - global_map_.info.origin.position.y) / gmap_res);

            if (gx < 0 || gy < 0 || gx >= gmap_w || gy >= gmap_h)
                continue;

            const int gmap_ind = gy * gmap_w + gx;
            global_map_.data[gmap_ind] = val;
        }
    }

    global_map_.header.stamp = rclcpp::Clock().now();
}

}
