#include "lidar_lane_detector/lidar_lane_detector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <Eigen/Dense>

LaneDetector::LaneDetector()
: Node("lidar_lane_detector")
{
  cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", rclcpp::SensorDataQoS(), std::bind(&LaneDetector::cloud_callback, this, std::placeholders::_1));
  roi_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/roi_filtered_points", 10);
  candidate_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/candidate_filtered_points", 10);
  cluster_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dbscan_clusters", 10);
  left_ransac_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/left_lane", 10);
  right_ransac_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/right_lane", 10);
  stop_ransac_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stop_ransac_cloud", 10);
  ransac_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/ransac_lane", 10);

  this->declare_parameter<double>("roi_x_min", 0.0);
  this->declare_parameter<double>("roi_x_max", 10.0);
  this->declare_parameter<double>("roi_y_min", -2.0);
  this->declare_parameter<double>("roi_y_max", 2.0);
  this->declare_parameter<double>("roi_z_min", -1.5);
  this->declare_parameter<double>("roi_z_max", -1.0);
  this->declare_parameter<int>("standard_intensity_threshold", 500);
  this->declare_parameter<int>("left_intensity_threshold", 500);
  this->declare_parameter<int>("right_intensity_threshold", 800);
  this->declare_parameter<double>("minimum_stop_line_length", 1.5);
  this->declare_parameter<double>("eps", 0.35);
  this->declare_parameter<int>("min_pts", 14);
}

void LaneDetector::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<PointT> cloud;
  pcl::PointCloud<PointT> roi_cloud;

  pcl::fromROSMsg(*cloud_msg, cloud);

  std::vector<VectorT> roi_points(1024);
  std::vector<VectorT> candidate_points(1024);

  VectorT left_lane;
  VectorT right_lane;
  VectorT stop_line;

  if (cloud.points.empty())
  {
    return;
  }

  set_roi(&cloud, &roi_points);                                                                // function
  extract_point(&roi_points, &candidate_points);
  std::vector<VectorT> clusters = dbscan(&candidate_points);
  publish_clusters(clusters, cloud_msg->header);
  separate_lane(clusters, left_lane, right_lane, stop_line);

  Ransac(left_lane, cloud_msg->header, "left_lane", 0);
  Ransac(right_lane, cloud_msg->header, "right_lane", 1);
  RansacStopLine(stop_line, cloud_msg->header, "stop_line", 2);

  roi_cloud.clear();                                                                           // 1) roi filtered cloud pub
  roi_cloud.points.reserve(cloud.points.size());

  for (const auto & scan_points : roi_points)
  {
    roi_cloud.points.insert(roi_cloud.points.end(), scan_points.begin(), scan_points.end());
  }

  roi_cloud.width = static_cast<uint32_t>(roi_cloud.points.size());
  roi_cloud.height = 1;
  roi_cloud.is_dense = false;
  roi_cloud.header = cloud.header;

  sensor_msgs::msg::PointCloud2 roi_cloud_msg;
  pcl::toROSMsg(roi_cloud, roi_cloud_msg);

  roi_cloud_msg.header = cloud_msg->header;
  roi_cloud_pub->publish(roi_cloud_msg);

  pcl::PointCloud<PointT> candidate_cloud;                                                    // 2) candidated cloud(based on intensity threshold) pub

  for (const auto & scan_points : candidate_points)
  {
    candidate_cloud.points.insert(candidate_cloud.points.end(), scan_points.begin(), scan_points.end());
  }

  candidate_cloud.width = static_cast<uint32_t>(candidate_cloud.points.size());
  candidate_cloud.height = 1;
  candidate_cloud.is_dense = false;
  candidate_cloud.header = cloud.header;

  sensor_msgs::msg::PointCloud2 candidate_cloud_msg;
  pcl::toROSMsg(candidate_cloud, candidate_cloud_msg);
  candidate_cloud_msg.header = cloud_msg->header;
  candidate_cloud_pub->publish(candidate_cloud_msg);
}

void LaneDetector::set_roi(const pcl::PointCloud<PointT> *cloud, std::vector<VectorT> *all_roi_points)
{
  double roi_x_min;
  double roi_x_max;
  double roi_y_min;
  double roi_y_max;
  double roi_z_min;
  double roi_z_max;

  this->get_parameter("roi_x_min", roi_x_min);
  this->get_parameter("roi_x_max", roi_x_max);
  this->get_parameter("roi_y_min", roi_y_min);
  this->get_parameter("roi_y_max", roi_y_max);
  this->get_parameter("roi_z_min", roi_z_min);
  this->get_parameter("roi_z_max", roi_z_max);

  // printf("ROI : x_min -> %.2lf, x_max -> %.2lf\n", roi_x_min, roi_x_max);
  // printf("ROI : y_min -> %.2lf, y_max -> %.2lf\n", roi_y_min, roi_y_max);
  // printf("ROI : z_min -> %.2lf, z_max -> %.2lf\n", roi_z_min, roi_z_max);

  for (auto & scan_points : *all_roi_points)
  {
    scan_points.clear();
  }

  size_t all_count = 0;

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    const auto & point = cloud->points[i];

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      continue;
    }

    if (point.x >= roi_x_min && point.x <= roi_x_max &&
        point.y >= roi_y_min && point.y <= roi_y_max &&
        point.z >= roi_z_min && point.z <= roi_z_max)
    {
      size_t scan_index = 0;
      
      if (cloud->height > 1 && cloud->width == all_roi_points->size())
      {
        scan_index = i % cloud->width;
      }
      else
      {
        scan_index = std::min(i / 32, all_roi_points->size() - 1);
      }

      (*all_roi_points)[scan_index].push_back(point);
      ++all_count;
    }
  }
}

void LaneDetector::extract_point(const std::vector<VectorT> *all_roi_points, std::vector<VectorT> *candidate_points)
{
  int standard_intensity_threshold;
  this->get_parameter("standard_intensity_threshold", standard_intensity_threshold);

	for (size_t a = 0; a < all_roi_points->size(); a++)
	{
		for (size_t i = 0; i < (*all_roi_points)[a].size(); i++)
		{
      if ((*all_roi_points)[a][i].intensity > standard_intensity_threshold)
      {
        int intensity = (*all_roi_points)[a][i].intensity;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "intensity: %zu", intensity);
        (*candidate_points)[a].push_back((*all_roi_points)[a][i]);
      }
		}
	}

	size_t road_mark_count = 0;
	for (const auto &v : *candidate_points)
	{
		road_mark_count += v.size();
	}

	// RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "candidate point size: %zu", road_mark_count);
}

std::vector<LaneDetector::VectorT> LaneDetector::dbscan(const std::vector<VectorT> *candidate_points)
{
  // RCLCPP_INFO(this->get_logger(), "candidate outer size = %zu", candidate_points->size());

  // for (size_t i = 0; i < candidate_points->size(); ++i)
  // {
  //   RCLCPP_INFO(this->get_logger(), "candidate_points[%zu] size = %zu", i, candidate_points->at(i).size());
  // }

  VectorT flattened_points;

  for (const auto & column_points : *candidate_points)
  {
    flattened_points.insert(flattened_points.end(), column_points.begin(), column_points.end());
  }
  // RCLCPP_INFO(this->get_logger(), "DBSCAN flattened points: %zu", flattened_points.size());

  if (flattened_points.empty())                                                   // if no points -> return
  {
    return {};
  }

  double eps;
  int min_pts;
  int cluster_id = 0;

  this->get_parameter("eps", eps);
  this->get_parameter("min_pts", min_pts);

  constexpr int UNCLASSIFIED = -2;                                                  // index (unclassified point: -2, noise point: -1)
  constexpr int NOISE = -1;

  std::vector<bool> is_visited(flattened_points.size(), false);                               // judge that the point is visited -> default = false         
  std::vector<int> labels(flattened_points.size(), UNCLASSIFIED);                             // label default = unclassified, (index default = -2)

  std::vector<std::vector<size_t>> neighbors(flattened_points.size());                        // neighbor point vector
  std::vector<bool> is_core_point(flattened_points.size(), false);                            // judge that the point is core point -> default = false

  VectorT core_points;
  VectorT border_points;
  VectorT noise_points;

  for (size_t i = 0; i < flattened_points.size(); ++i)
  { 
    for (size_t j = 0; j < flattened_points.size(); ++j)
    {
      const double distance = calculate_distance(flattened_points[i].x, flattened_points[i].y, flattened_points[j].x, flattened_points[j].y);     // calculate the distance between two points

      if (distance <= eps)                                                                                // if the distance is under eps -> neighbors point
      {                                                                                                   // -> judge all distance and put the points in neighbors vector
        neighbors[i].push_back(j);
      }
    }

    if (neighbors[i].size() >= static_cast<size_t>(min_pts))                                              // if neighbors vector has points more than min_pts's value
    {                                                                                                     // -> this point is core point, and this point put into core_points vector
      is_core_point[i] = true;
      core_points.push_back(flattened_points[i]);
    }
  }

  for (size_t i = 0; i < flattened_points.size(); ++i)
  {
    if (is_core_point[i])                                                                                 // if this point is core point -> pass
    {
      continue;
    }

    bool has_core_neighbor = false;

    for (const size_t neighbor_index : neighbors[i])                                                      // neighbors point
    {
      if (is_core_point[neighbor_index])                                                                  // if the not core point has core point nearby 
      {
        has_core_neighbor = true;                                                                         // -> has core point nearby :true and break
        break;
      }
    }

    if (has_core_neighbor)                                                                                // just have core point nearby -> border point(this point is not a core point, but it has core point nearby -> border point)
    {
      border_points.push_back(flattened_points[i]);
    }
    else
    {
      noise_points.push_back(flattened_points[i]);                                                        // this point is not a core point, and dont have core point nearby -> noise point(label = -1)
      labels[i] = NOISE;
    }
  }

  for (size_t i = 0; i < flattened_points.size(); ++i)
  {
    if (!is_core_point[i])                                                                                // if this point is not core point -> pass
    {
      continue;
    }

    if (labels[i] != UNCLASSIFIED)                                                                        // already labeled core point -> pass(dont clust new group)
    {
      continue;
    }

    labels[i] = cluster_id;
    is_visited[i] = true;

    std::queue<size_t> search_queue;
    for (const size_t neighbor_index : neighbors[i])
    {
      search_queue.push(neighbor_index);                                                                  // array neighbors point in a row
    }

    while (!search_queue.empty())                                                                         // until there is no point in neighbors vector
    {
      const size_t current_index = search_queue.front();                                                  // first point -> current index
      search_queue.pop();                                                                                 // delete first point in queue

      if (!is_visited[current_index])                                                                     // if current index point has not been visited yet -> process it 
      {                                                                                                   // if this point marked as visited -> do not process this point again
        is_visited[current_index] = true;

        if (is_core_point[current_index])                                                                 // and if it is core point
        {
          for (const size_t next_neighbor_index : neighbors[current_index])
          {
            if (labels[next_neighbor_index] == UNCLASSIFIED || labels[next_neighbor_index] == NOISE)      // if next point(this point's neighbor points) -> not classified or noise point before
            {
              search_queue.push(next_neighbor_index);                                                     // array that point in search_queue
            }
          }
        }
      }

      if (labels[current_index] == UNCLASSIFIED || labels[current_index] == NOISE)
      {
        labels[current_index] = cluster_id;                                                            // cluster 0, 1, 2 .. -> make new clusters
      }
    }

    ++cluster_id;
  }

  std::vector<VectorT> clusters(static_cast<size_t>(cluster_id));
  for (size_t i = 0; i < flattened_points.size(); ++i)
  {
    if (labels[i] >= 0)
    {
      clusters[static_cast<size_t>(labels[i])].push_back(flattened_points[i]);
    }
  }

  // RCLCPP_INFO(this->get_logger(), "DBSCAN input points=%zu, cluster count=%zu", flattened_points.size(),clusters.size());

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    // RCLCPP_INFO(this->get_logger(), "cluster[%zu] size=%zu", i, clusters[i].size());
  }

  if (clusters.empty())
  {
    return {};
  }

  return clusters;
}

void LaneDetector::publish_clusters(const std::vector<VectorT> &clusters, const std_msgs::msg::Header &header)
{
  pcl::PointCloud<PointT> cluster_cloud;

  cluster_cloud.header.frame_id = header.frame_id;
  cluster_cloud.height = 1;
  cluster_cloud.is_dense = false;

  for (size_t cluster_index = 0;
       cluster_index < clusters.size();
       ++cluster_index)
  {
    for (const auto & original_point : clusters[cluster_index])
    {
      PointT point = original_point;
      point.intensity = static_cast<float>((cluster_index + 1) * 30);
      cluster_cloud.points.push_back(point);
    }
  }
  cluster_cloud.width = static_cast<uint32_t>(cluster_cloud.points.size());
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(cluster_cloud, output_msg);
  output_msg.header = header;

  cluster_cloud_pub->publish(output_msg);
}

void LaneDetector::separate_lane(const std::vector<VectorT> &clusters, VectorT &left_lane, VectorT &right_lane, VectorT &stop_line)
{
  left_lane.clear();
  right_lane.clear();
  stop_line.clear();

  int left_intensity_threshold;
  int right_intensity_threshold;

  this->get_parameter("left_intensity_threshold", left_intensity_threshold);
  this->get_parameter("right_intensity_threshold", right_intensity_threshold);

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    const VectorT & cluster = clusters[i];

    if (cluster.size() < 3)
    {
      continue;
    }

    double mean_x = 0.0;
    double mean_y = 0.0;

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto & point : cluster)
    {
      mean_x += point.x;
      mean_y += point.y;

      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));

      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }

    mean_x /= static_cast<double>(cluster.size());
    mean_y /= static_cast<double>(cluster.size());

    const double x_span = max_x - min_x;
    const double y_span = max_y - min_y;

    if (y_span > x_span * 1.5)
    {
      if (cluster.size() > stop_line.size())
      {
        stop_line = cluster;
      }

      continue;
    }

    if (x_span <= y_span * 1.5)
    {
      continue;
    }


    if (mean_y > 0.0)
    {
      VectorT filtered_left;
      filtered_left.reserve(cluster.size());

      for (const auto & point : cluster)
      {
        if (point.intensity > static_cast<float>(left_intensity_threshold))
        {
          // int intensity = point.intensity;
          // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "intensity: %zu", intensity);
          filtered_left.push_back(point);
        }
      }

      // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      //   "LEFT cluster[%zu]: threshold=%d, passed=%zu/%zu (%.1f%%)",
      //   i, left_intensity_threshold, filtered_left.size(), cluster.size(),
      //   100.0 * static_cast<double>(filtered_left.size()) / cluster.size());

      if (filtered_left.size() > left_lane.size())
      {
        left_lane = std::move(filtered_left);
      }
    }

    else if (mean_y < 0.0)
    {
      VectorT filtered_right;
      filtered_right.reserve(cluster.size());

      for (const auto & point : cluster)
      {
        if (point.intensity > static_cast<float>(right_intensity_threshold))
        {
          // int intensity = point.intensity;
          // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "right intensity: %zu", intensity);
          filtered_right.push_back(point);
        }
      }

      // RCLCPP_INFO_THROTTLE(
      //   this->get_logger(), *this->get_clock(), 1000,
      //   "RIGHT cluster[%zu]: threshold=%d, passed=%zu/%zu (%.1f%%)",
      //   i, right_intensity_threshold, filtered_right.size(), cluster.size(),
      //   100.0 * static_cast<double>(filtered_right.size()) / cluster.size());

      if (filtered_right.size() > right_lane.size())
      {
        right_lane = std::move(filtered_right);
      }
    }

    // RCLCPP_INFO(this->get_logger(), "cluster[%zu]: mean=(%.2f, %.2f), " "span=(%.2f, %.2f)", i, mean_x, mean_y, x_span, y_span);
  }

  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Final points: left=%zu, right=%zu, stop=%zu", left_lane.size(), right_lane.size(), stop_line.size());
}

void LaneDetector::Ransac(const VectorT &lane_points, const std_msgs::msg::Header & header, const std::string & marker_namespace, const int marker_id)
{
  if (lane_points.size() < 2)
  {
    return;
  }

  static std::random_device rd;
  static std::mt19937 generator(rd());

  std::uniform_int_distribution<size_t> distribution(0, lane_points.size() - 1);

  const int max_iterations = 300;
  const double distance_threshold = 0.12;
  const double min_sample_x_span = 0.3;
  const size_t min_inlier_count = std::max<size_t>(3, lane_points.size() / 5);

  size_t best_inlier_count = 0;
  double best_a = 0.0;
  double best_b = 0.0;
  VectorT best_inliers;

  for (int iteration = 0; iteration < max_iterations; ++iteration)
  {
    size_t index1 = distribution(generator);
    size_t index2 = distribution(generator);

    while (index2 == index1)
    {
      index2 = distribution(generator);
    }

    const auto & point1 = lane_points[index1];
    const auto & point2 = lane_points[index2];

    const double sample_min_x = std::min({static_cast<double>(point1.x), static_cast<double>(point2.x)});
    const double sample_max_x = std::max({static_cast<double>(point1.x), static_cast<double>(point2.x)});

    if (sample_max_x - sample_min_x < min_sample_x_span)
    {
      continue;
    }

    const double dx = point2.x - point1.x;
    if (std::abs(dx) < 1e-6)
    {
      continue;
    }

    const double a = (point2.y - point1.y) / dx;
    const double b = point1.y - a * point1.x;

    size_t current_inlier_count = 0;
    VectorT current_inliers;

    for (const auto & point : lane_points)
    {
      const double error = std::abs(a * point.x - point.y + b) / std::sqrt(a * a + 1.0);

      if (error < distance_threshold)
      {
        ++current_inlier_count;
        current_inliers.push_back(point);
      }
    }

    if (current_inlier_count > best_inlier_count)
    {
      best_inlier_count = current_inlier_count;

      best_a = a;
      best_b = b;

      best_inliers = current_inliers;
    }
  }

  if (best_inliers.size() < min_inlier_count)
  {
    // RCLCPP_INFO(this->get_logger(),"Linear RANSAC rejected: inliers=%zu/%zu", best_inliers.size(), lane_points.size());
    return;
  }

  Eigen::MatrixXd refit_matrix(best_inliers.size(), 2);
  Eigen::VectorXd refit_y(best_inliers.size());
  for (size_t i = 0; i < best_inliers.size(); ++i)
  {
    const double x = best_inliers[i].x;
    refit_matrix(static_cast<Eigen::Index>(i), 0) = x;
    refit_matrix(static_cast<Eigen::Index>(i), 1) = 1.0;
    refit_y(static_cast<Eigen::Index>(i)) = best_inliers[i].y;
  }

  const Eigen::Vector2d refit_coefficients =
    refit_matrix.colPivHouseholderQr().solve(refit_y);

  best_a = refit_coefficients(0);
  best_b = refit_coefficients(1);

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double z_sum = 0.0;

  for (const auto & point : best_inliers)
  {
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    z_sum += point.z;
  }

  if (max_x <= min_x)
  {
    return;
  }

  if (marker_namespace == "left_lane")
  {
    publish_ransac_cloud(best_inliers, header, left_ransac_cloud_pub);
  }
  else if (marker_namespace == "right_lane")
  {
    publish_ransac_cloud(best_inliers, header, right_ransac_cloud_pub);
  }

  visualization_msgs::msg::Marker ransac_line;
  ransac_line.header = header;
  ransac_line.ns = marker_namespace;
  ransac_line.id = marker_id;
  ransac_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  ransac_line.action = visualization_msgs::msg::Marker::ADD;
  ransac_line.pose.orientation.w = 1.0;
  ransac_line.scale.x = 0.06;
  ransac_line.color.a = 1.0f;

  if (marker_namespace.find("left") != std::string::npos)
  {
    ransac_line.color.r = 0.1f;
    ransac_line.color.g = 1.0f;
    ransac_line.color.b = 0.1f;
  }
  else
  {
    ransac_line.color.r = 1.0f;
    ransac_line.color.g = 0.2f;
    ransac_line.color.b = 0.1f;
  }

  ransac_line.lifetime.sec = 1;
  ransac_line.lifetime.nanosec = 0;

  const double average_z = z_sum / static_cast<double>(best_inliers.size());
  constexpr size_t sample_count = 30;
  ransac_line.points.reserve(sample_count);

  for (size_t i = 0; i < sample_count; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(sample_count - 1);
    const double x = min_x + ratio * (max_x - min_x);
    const double y = best_a * x + best_b;

    geometry_msgs::msg::Point line_point;
    line_point.x = x;
    line_point.y = y;
    line_point.z = average_z;
    ransac_line.points.push_back(line_point);
  }

  ransac_marker_pub->publish(ransac_line);
}

void LaneDetector::RansacStopLine(const VectorT & stop_points, const std_msgs::msg::Header & header, const std::string & marker_namespace, const int marker_id)
{
  if (stop_points.size() < 2)
  {
    return;
  }

  static std::random_device random_device;
  static std::mt19937 generator(random_device());

  std::uniform_int_distribution<size_t> distribution(0, stop_points.size() - 1);

  constexpr int max_iterations = 300;
  constexpr double distance_threshold = 0.12;
  constexpr double minimum_sample_y_span = 0.3;

  const size_t minimum_inlier_count = std::max<size_t>(8, stop_points.size() / 8);

  size_t best_inlier_count = 0;
  VectorT best_inliers;

  for (int iteration = 0; iteration < max_iterations; ++iteration)
  {
    const size_t index1 = distribution(generator);
    size_t index2 = distribution(generator);

    while (index2 == index1)
    {
      index2 = distribution(generator);
    }

    const PointT & point1 = stop_points[index1];
    const PointT & point2 = stop_points[index2];

    const double dy = static_cast<double>(point2.y) - static_cast<double>(point1.y);

    if (std::abs(dy) < minimum_sample_y_span)
    {
      continue;
    }

    const double a = (static_cast<double>(point2.x) - static_cast<double>(point1.x)) / dy;

    const double b = static_cast<double>(point1.x) - a * static_cast<double>(point1.y);

    VectorT current_inliers;
    current_inliers.reserve(stop_points.size());

    for (const PointT & point : stop_points)
    {
      // x = ay + b
      const double distance = std::abs(static_cast<double>(point.x) - a*static_cast<double>(point.y) - b) / std::sqrt(a * a + 1.0);

      if (distance <= distance_threshold)
      {
        current_inliers.push_back(point);
      }
    }

    if (current_inliers.size() > best_inlier_count)
    {
      best_inlier_count = current_inliers.size();
      best_inliers = std::move(current_inliers);
    }
  }

  if (best_inliers.size() < minimum_inlier_count)
  {
    return;
  }

  Eigen::MatrixXd matrix(best_inliers.size(), 2);
  Eigen::VectorXd output(best_inliers.size());

  for (size_t i = 0; i < best_inliers.size(); ++i)
  {
    matrix(static_cast<Eigen::Index>(i), 0) = best_inliers[i].y;
    matrix(static_cast<Eigen::Index>(i), 1) = 1.0;

    output(static_cast<Eigen::Index>(i)) = best_inliers[i].x;
  }

  const Eigen::Vector2d coefficients = matrix.colPivHouseholderQr().solve(output);

  const double a = coefficients[0];
  const double b = coefficients[1];

  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  double z_sum = 0.0;

  for (const PointT & point : best_inliers)
  {
    min_y = std::min(min_y, static_cast<double>(point.y));

    max_y = std::max(max_y, static_cast<double>(point.y));

    z_sum += point.z;
  }

  double stop_line_length = max_y - min_y;
  double minimum_stop_line_length;
  this->get_parameter("minimum_stop_line_length", minimum_stop_line_length);

  if (stop_line_length < minimum_stop_line_length)
  {
    // RCLCPP_INFO(this->get_logger(), "Stop line rejected: length=%.2f m, minimum=%.2f m", stop_line_length, minimum_stop_line_length);
    return;
  }

  publish_ransac_cloud(best_inliers, header, stop_ransac_cloud_pub);

  const double average_z = z_sum / static_cast<double>(best_inliers.size());

  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.09;

  marker.color.r = 0.0f;
  marker.color.g = 0.4f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  constexpr size_t sample_count = 30;
  marker.points.reserve(sample_count);

  for (size_t i = 0; i < sample_count; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(sample_count - 1);

    const double y = min_y + ratio * (max_y - min_y);
    const double x = a * y + b;

    geometry_msgs::msg::Point marker_point;
    marker_point.x = x;
    marker_point.y = y;
    marker_point.z = average_z + 0.05;

    marker.points.push_back(marker_point);
  }

  ransac_marker_pub->publish(marker);
}

void LaneDetector::publish_ransac_cloud(
  const VectorT &points,
  const std_msgs::msg::Header &header,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header = header;
  message.height = 1;
  message.width = static_cast<uint32_t>(points.size());
  message.is_bigendian = false;
  message.is_dense = false;

  message.fields.resize(4);

  message.fields[0].name = "x";
  message.fields[0].offset = 0;
  message.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  message.fields[0].count = 1;

  message.fields[1].name = "y";
  message.fields[1].offset = 4;
  message.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  message.fields[1].count = 1;

  message.fields[2].name = "z";
  message.fields[2].offset = 8;
  message.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  message.fields[2].count = 1;

  message.fields[3].name = "intensity";
  message.fields[3].offset = 12;
  message.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  message.fields[3].count = 1;

  message.point_step = 16;
  message.row_step = message.point_step * message.width;
  message.data.resize(message.row_step * message.height);

  for (size_t i = 0; i < points.size(); ++i) {
    float values[4] = {
      points[i].x,
      points[i].y,
      points[i].z,
      points[i].intensity
    };

    std::memcpy(&message.data[i * message.point_step], values, message.point_step);
  }

  publisher->publish(message);
}

double LaneDetector::calculate_distance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x1 - x2, y1 - y2);
}