# livox_bag_recorder

用于录制livox mid360雷达的数据
- `/livox/lidar`
- `/livox/lidar/pointcloud2`
- `/livox/imu`

## bag 保存目录
当前包下的`bag_files`目录，每个bag的命名为`rosbag2_YYYY-MM-DD-HH-MM-SS`，例如 *rosbag2_2024-08-11-20-29-22*

## 播放bag
```bash
ros2 bag play <bag name>
```