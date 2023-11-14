ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked &
ros2 run ros_gz_bridge parameter_bridge /world/ieee_maze_1/model/x500_lidar_1/link/base_link/sensor/imu_sensor/Imu@sensor_msgs/msg/Imu@gz.msgs.IMU &
ros2 run ros_gz_bridge parameter_bridge /px4_1/fmu/out/vehicle_odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry
