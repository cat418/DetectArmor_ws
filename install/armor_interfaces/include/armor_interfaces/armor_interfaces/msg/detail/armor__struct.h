// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from armor_interfaces:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef ARMOR_INTERFACES__MSG__DETAIL__ARMOR__STRUCT_H_
#define ARMOR_INTERFACES__MSG__DETAIL__ARMOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
// Member 'color'
#include "rosidl_runtime_c/string.h"
// Member 'apexs'
#include "geometry_msgs/msg/detail/point32__struct.h"
// Member 'pose'
// Member 'world_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/Armor in the package armor_interfaces.
/**
  * 无符号8位整数 
 */
typedef struct armor_interfaces__msg__Armor
{
  uint8_t number;
  /// 字符串类型
  rosidl_runtime_c__String type;
  rosidl_runtime_c__String color;
  /// 32位浮动小数类型 距离中心的距离
  float distance_to_center;
  /// 包含四个特征点的的消息类型 []表示这是一个数组 存储多个坐标点
  geometry_msgs__msg__Point32 apexs[4];
  /// ROS中表示位置和姿态的消息类型
  geometry_msgs__msg__Pose pose;
  /// 世界坐标系 姿态
  geometry_msgs__msg__Pose world_pose;
} armor_interfaces__msg__Armor;

// Struct for a sequence of armor_interfaces__msg__Armor.
typedef struct armor_interfaces__msg__Armor__Sequence
{
  armor_interfaces__msg__Armor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} armor_interfaces__msg__Armor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARMOR_INTERFACES__MSG__DETAIL__ARMOR__STRUCT_H_
