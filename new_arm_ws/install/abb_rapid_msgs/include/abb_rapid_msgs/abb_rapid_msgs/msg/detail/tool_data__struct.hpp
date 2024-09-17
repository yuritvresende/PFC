// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from abb_rapid_msgs:msg/ToolData.idl
// generated code does not contain a copyright notice

#ifndef ABB_RAPID_MSGS__MSG__DETAIL__TOOL_DATA__STRUCT_HPP_
#define ABB_RAPID_MSGS__MSG__DETAIL__TOOL_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tframe'
#include "abb_rapid_msgs/msg/detail/pose__struct.hpp"
// Member 'tload'
#include "abb_rapid_msgs/msg/detail/load_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__abb_rapid_msgs__msg__ToolData __attribute__((deprecated))
#else
# define DEPRECATED__abb_rapid_msgs__msg__ToolData __declspec(deprecated)
#endif

namespace abb_rapid_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ToolData_
{
  using Type = ToolData_<ContainerAllocator>;

  explicit ToolData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tframe(_init),
    tload(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robhold = false;
    }
  }

  explicit ToolData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tframe(_alloc, _init),
    tload(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robhold = false;
    }
  }

  // field types and members
  using _robhold_type =
    bool;
  _robhold_type robhold;
  using _tframe_type =
    abb_rapid_msgs::msg::Pose_<ContainerAllocator>;
  _tframe_type tframe;
  using _tload_type =
    abb_rapid_msgs::msg::LoadData_<ContainerAllocator>;
  _tload_type tload;

  // setters for named parameter idiom
  Type & set__robhold(
    const bool & _arg)
  {
    this->robhold = _arg;
    return *this;
  }
  Type & set__tframe(
    const abb_rapid_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->tframe = _arg;
    return *this;
  }
  Type & set__tload(
    const abb_rapid_msgs::msg::LoadData_<ContainerAllocator> & _arg)
  {
    this->tload = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    abb_rapid_msgs::msg::ToolData_<ContainerAllocator> *;
  using ConstRawPtr =
    const abb_rapid_msgs::msg::ToolData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<abb_rapid_msgs::msg::ToolData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<abb_rapid_msgs::msg::ToolData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      abb_rapid_msgs::msg::ToolData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<abb_rapid_msgs::msg::ToolData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      abb_rapid_msgs::msg::ToolData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<abb_rapid_msgs::msg::ToolData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<abb_rapid_msgs::msg::ToolData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<abb_rapid_msgs::msg::ToolData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__abb_rapid_msgs__msg__ToolData
    std::shared_ptr<abb_rapid_msgs::msg::ToolData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__abb_rapid_msgs__msg__ToolData
    std::shared_ptr<abb_rapid_msgs::msg::ToolData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolData_ & other) const
  {
    if (this->robhold != other.robhold) {
      return false;
    }
    if (this->tframe != other.tframe) {
      return false;
    }
    if (this->tload != other.tload) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolData_

// alias to use template instance with default allocator
using ToolData =
  abb_rapid_msgs::msg::ToolData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace abb_rapid_msgs

#endif  // ABB_RAPID_MSGS__MSG__DETAIL__TOOL_DATA__STRUCT_HPP_
