from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lidar_lane_detector",
            executable="lidar_lane_detector",
            name="lidar_lane_detector",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "roi_x_min" : 0.0,  # -6.0
                "roi_x_max" : 15.0, # 0.0
                "roi_y_min" : -3.5,
                "roi_y_max" : 3.5,
                "roi_z_min" : -1.5,
                "roi_z_max" : -1.0,
                "standard_intensity_threshold" : 2900,
                "left_intensity_threshold" : 6500,
                "right_intensity_threshold" : 6500,
                "minimum_stop_line_length" : 1.5,
                "eps" : 1.0,
                "min_pts" : 3
            }]
        )
    ])
   
