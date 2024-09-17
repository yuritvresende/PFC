// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from abb_robot_msgs:msg/MechanicalUnitState.idl
// generated code does not contain a copyright notice
#include "abb_robot_msgs/msg/detail/mechanical_unit_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
abb_robot_msgs__msg__MechanicalUnitState__init(abb_robot_msgs__msg__MechanicalUnitState * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    abb_robot_msgs__msg__MechanicalUnitState__fini(msg);
    return false;
  }
  // activated
  return true;
}

void
abb_robot_msgs__msg__MechanicalUnitState__fini(abb_robot_msgs__msg__MechanicalUnitState * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // activated
}

bool
abb_robot_msgs__msg__MechanicalUnitState__are_equal(const abb_robot_msgs__msg__MechanicalUnitState * lhs, const abb_robot_msgs__msg__MechanicalUnitState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // activated
  if (lhs->activated != rhs->activated) {
    return false;
  }
  return true;
}

bool
abb_robot_msgs__msg__MechanicalUnitState__copy(
  const abb_robot_msgs__msg__MechanicalUnitState * input,
  abb_robot_msgs__msg__MechanicalUnitState * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // activated
  output->activated = input->activated;
  return true;
}

abb_robot_msgs__msg__MechanicalUnitState *
abb_robot_msgs__msg__MechanicalUnitState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  abb_robot_msgs__msg__MechanicalUnitState * msg = (abb_robot_msgs__msg__MechanicalUnitState *)allocator.allocate(sizeof(abb_robot_msgs__msg__MechanicalUnitState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(abb_robot_msgs__msg__MechanicalUnitState));
  bool success = abb_robot_msgs__msg__MechanicalUnitState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
abb_robot_msgs__msg__MechanicalUnitState__destroy(abb_robot_msgs__msg__MechanicalUnitState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    abb_robot_msgs__msg__MechanicalUnitState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
abb_robot_msgs__msg__MechanicalUnitState__Sequence__init(abb_robot_msgs__msg__MechanicalUnitState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  abb_robot_msgs__msg__MechanicalUnitState * data = NULL;

  if (size) {
    data = (abb_robot_msgs__msg__MechanicalUnitState *)allocator.zero_allocate(size, sizeof(abb_robot_msgs__msg__MechanicalUnitState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = abb_robot_msgs__msg__MechanicalUnitState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        abb_robot_msgs__msg__MechanicalUnitState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
abb_robot_msgs__msg__MechanicalUnitState__Sequence__fini(abb_robot_msgs__msg__MechanicalUnitState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      abb_robot_msgs__msg__MechanicalUnitState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

abb_robot_msgs__msg__MechanicalUnitState__Sequence *
abb_robot_msgs__msg__MechanicalUnitState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  abb_robot_msgs__msg__MechanicalUnitState__Sequence * array = (abb_robot_msgs__msg__MechanicalUnitState__Sequence *)allocator.allocate(sizeof(abb_robot_msgs__msg__MechanicalUnitState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = abb_robot_msgs__msg__MechanicalUnitState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
abb_robot_msgs__msg__MechanicalUnitState__Sequence__destroy(abb_robot_msgs__msg__MechanicalUnitState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    abb_robot_msgs__msg__MechanicalUnitState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
abb_robot_msgs__msg__MechanicalUnitState__Sequence__are_equal(const abb_robot_msgs__msg__MechanicalUnitState__Sequence * lhs, const abb_robot_msgs__msg__MechanicalUnitState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!abb_robot_msgs__msg__MechanicalUnitState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
abb_robot_msgs__msg__MechanicalUnitState__Sequence__copy(
  const abb_robot_msgs__msg__MechanicalUnitState__Sequence * input,
  abb_robot_msgs__msg__MechanicalUnitState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(abb_robot_msgs__msg__MechanicalUnitState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    abb_robot_msgs__msg__MechanicalUnitState * data =
      (abb_robot_msgs__msg__MechanicalUnitState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!abb_robot_msgs__msg__MechanicalUnitState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          abb_robot_msgs__msg__MechanicalUnitState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!abb_robot_msgs__msg__MechanicalUnitState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
