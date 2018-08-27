# frontal_delphi_radar
This is a ros package for Delphi Millimeter Wave Radar(MMW) data parsing and visualizing.
## MMW target properties definition
For every single radar target, we have define below properties:
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
Obtain and parse radar CAN data, and publish the radar messages related topics
#### Subscribed Topics
- **`/ecu_data`**([std_msgs::Float32])
- **`/imudata`**([sensor_msgs::Imu])
#### Published Topics
- **`radardata`**([frontal_delphi_radar::RadarData])
### Node: frontal_delphi_radar（main.cpp)
Radar targets visualizer
#### Subscribed Topics
- **`/radardata`**([frontal_delphi_radar::RadarData])
#### Published Topics

