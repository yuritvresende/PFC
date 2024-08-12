// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robotiq_85_msgs:msg/GripperCmd.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "robotiq_85_msgs/msg/detail/gripper_cmd__struct.h"
#include "robotiq_85_msgs/msg/detail/gripper_cmd__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robotiq_85_msgs__msg__gripper_cmd__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robotiq_85_msgs.msg._gripper_cmd.GripperCmd", full_classname_dest, 43) == 0);
  }
  robotiq_85_msgs__msg__GripperCmd * ros_message = _ros_message;
  {  // emergency_release
    PyObject * field = PyObject_GetAttrString(_pymsg, "emergency_release");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->emergency_release = (Py_True == field);
    Py_DECREF(field);
  }
  {  // emergency_release_dir
    PyObject * field = PyObject_GetAttrString(_pymsg, "emergency_release_dir");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->emergency_release_dir = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // stop
    PyObject * field = PyObject_GetAttrString(_pymsg, "stop");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stop = (Py_True == field);
    Py_DECREF(field);
  }
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // force
    PyObject * field = PyObject_GetAttrString(_pymsg, "force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->force = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robotiq_85_msgs__msg__gripper_cmd__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GripperCmd */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robotiq_85_msgs.msg._gripper_cmd");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GripperCmd");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robotiq_85_msgs__msg__GripperCmd * ros_message = (robotiq_85_msgs__msg__GripperCmd *)raw_ros_message;
  {  // emergency_release
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->emergency_release ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "emergency_release", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // emergency_release_dir
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->emergency_release_dir);
    {
      int rc = PyObject_SetAttrString(_pymessage, "emergency_release_dir", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stop
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stop ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
