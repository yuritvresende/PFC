// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from abb_robot_msgs:msg/MechanicalUnitState.idl
// generated code does not contain a copyright notice

#ifndef ABB_ROBOT_MSGS__MSG__DETAIL__MECHANICAL_UNIT_STATE__STRUCT_HPP_
#define ABB_ROBOT_MSGS__MSG__DETAIL__MECHANICAL_UNIT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__abb_robot_msgs__msg__MechanicalUnitState __attribute__((deprecated))
#else
# define DEPRECATED__abb_robot_msgs__msg__MechanicalUnitState __declspec(deprecated)
#endif

namespace abb_robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MechanicalUnitState_
{
  using Type = MechanicalUnitState_<ContainerAllocator>;

  explicit MechanicalUnitState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->activated = false;
    }
  }

  explicit MechanicalUnitState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->activated = false;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _activated_type =
    bool;
  _activated_type activated;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__activated(
    const bool & _arg)
  {
    this->activated = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator> *;
  using ConstRawPtr =
    const abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__abb_robot_msgs__msg__MechanicalUnitState
    std::shared_ptr<abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__abb_robot_msgs__msg__MechanicalUnitState
    std::shared_ptr<abb_robot_msgs::msg::MechanicalUnitState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MechanicalUnitState_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->activated != other.activated) {
      return false;
    }
    return true;
  }
  bool operator!=(const MechanicalUnitState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MechanicalUnitState_

// alias to use template instance with default allocator
using MechanicalUnitState =
  abb_robot_msgs::msg::MechanicalUnitState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace abb_robot_msgs

#endif  // ABB_ROBOT_MSGS__MSG__DETAIL__MECHANICAL_UNIT_STATE__STRUCT_HPP_
