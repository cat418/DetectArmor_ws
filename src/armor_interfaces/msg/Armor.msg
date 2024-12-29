#无符号8位整数 
uint8 number
#字符串类型
string type
string color
#32位浮动小数类型 距离中心的距离
float32 distance_to_center
#包含四个特征点的的消息类型 []表示这是一个数组 存储多个坐标点
geometry_msgs/Point32[4] apexs
#ROS中表示位置和姿态的消息类型
geometry_msgs/Pose pose
#世界坐标系 姿态
geometry_msgs/Pose world_pose
