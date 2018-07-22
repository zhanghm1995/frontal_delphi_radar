# frontal_delphi_radar
## 毫米波雷达目标属性含义
对于每一个雷达目标点，有如下属性
`target_ID`
`range`
`v`
`angle`
`x`
`y`
`x`
`valid`
`status`
`moving`
`moving_fast`
`moving_slow`

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

