# frontal_delphi_radar

## ROS related
## Nodes
### Node: get_radar_data(get_radar_data.cpp)
获取并解析雷达数据，发布毫米波雷达数据话题
#### Subscribed Topics
- **`/ecu_data`**([std_msgs::Float32])
- **`/imudata`**([sensor_msgs::Imu])
#### Published Topics
- **`radardata`**([frontal_delphi_radar::RadarData])
### Node: frontal_delphi_radar（main.cpp)
雷达点可视化
#### Subscribed Topics
- **`/radardata`**([frontal_delphi_radar::RadarData])
#### Published Topics

