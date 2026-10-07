#ifndef LIDAR_LANE_DETECTOR_HPP
#define LIDAR_LANE_DETECTOR_HPP

#define PCL_NO_PRECOMPILE

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

// #include "point_os1.hpp"

#define _USE_MATH_DEFINES

class LaneDetector : public rclcpp::Node
{
	public:
		LaneDetector();

		using PointT = pcl::PointXYZI;
		using VectorT = std::vector<PointT>;

	private:
		struct Line
		{
			float a;
			float b;
		};

		

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_cloud_pub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr candidate_cloud_pub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_pub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_ransac_cloud_pub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_ransac_cloud_pub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stop_ransac_cloud_pub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ransac_marker_pub;

		void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
		void set_roi(const pcl::PointCloud<PointT> *cloud, std::vector<VectorT> *all_roi_points);
		void extract_point(const std::vector<VectorT> *roi_points, std::vector<VectorT> *candidate_points);
		std::vector<VectorT> dbscan(const std::vector<VectorT> *candidate_points);
		void separate_lane(const std::vector<VectorT> &clusters, VectorT &left_lane, VectorT &right_lane, VectorT &stop_line);
		void publish_clusters(const std::vector<VectorT> & clusters,const std_msgs::msg::Header & header);
		void Ransac(const VectorT &lane_points, const std_msgs::msg::Header & header, const std::string & marker_namespace, const int marker_id);
		void RansacStopLine(const VectorT & stop_points, const std_msgs::msg::Header & header, const std::string & marker_namespace, const int marker_id);
		void publish_ransac_cloud(const VectorT &points, const std_msgs::msg::Header &header, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher);
		double calculate_distance(double x1, double y1, double x2, double y2);
};

#endif
