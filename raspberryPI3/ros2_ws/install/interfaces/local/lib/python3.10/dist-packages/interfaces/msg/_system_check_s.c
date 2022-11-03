// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from interfaces:msg/SystemCheck.idl
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
#include "interfaces/msg/detail/system_check__struct.h"
#include "interfaces/msg/detail/system_check__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool interfaces__msg__system_check__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
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
    assert(strncmp("interfaces.msg._system_check.SystemCheck", full_classname_dest, 40) == 0);
  }
  interfaces__msg__SystemCheck * ros_message = _ros_message;
  {  // request
    PyObject * field = PyObject_GetAttrString(_pymsg, "request");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->request = (Py_True == field);
    Py_DECREF(field);
  }
  {  // response
    PyObject * field = PyObject_GetAttrString(_pymsg, "response");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->response = (Py_True == field);
    Py_DECREF(field);
  }
  {  // report
    PyObject * field = PyObject_GetAttrString(_pymsg, "report");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->report = (Py_True == field);
    Py_DECREF(field);
  }
  {  // print
    PyObject * field = PyObject_GetAttrString(_pymsg, "print");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->print = (Py_True == field);
    Py_DECREF(field);
  }
  {  // jetson
    PyObject * field = PyObject_GetAttrString(_pymsg, "jetson");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->jetson = (Py_True == field);
    Py_DECREF(field);
  }
  {  // l476
    PyObject * field = PyObject_GetAttrString(_pymsg, "l476");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->l476 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // f103
    PyObject * field = PyObject_GetAttrString(_pymsg, "f103");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->f103 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // comm_jetson
    PyObject * field = PyObject_GetAttrString(_pymsg, "comm_jetson");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->comm_jetson, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // comm_l476
    PyObject * field = PyObject_GetAttrString(_pymsg, "comm_l476");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->comm_l476, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // comm_f103
    PyObject * field = PyObject_GetAttrString(_pymsg, "comm_f103");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->comm_f103, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // battery
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->battery, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // ultrasonics
    PyObject * field = PyObject_GetAttrString(_pymsg, "ultrasonics");
    if (!field) {
      return false;
    }
    {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'ultrasonics'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = 6;
      rosidl_runtime_c__String * dest = ros_message->ultrasonics;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyUnicode_Check(item));
        PyObject * encoded_item = PyUnicode_AsUTF8String(item);
        if (!encoded_item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        rosidl_runtime_c__String__assign(&dest[i], PyBytes_AS_STRING(encoded_item));
        Py_DECREF(encoded_item);
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // gps
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->gps, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->imu, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // lidar
    PyObject * field = PyObject_GetAttrString(_pymsg, "lidar");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->lidar, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // camera
    PyObject * field = PyObject_GetAttrString(_pymsg, "camera");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->camera, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * interfaces__msg__system_check__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SystemCheck */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("interfaces.msg._system_check");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SystemCheck");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  interfaces__msg__SystemCheck * ros_message = (interfaces__msg__SystemCheck *)raw_ros_message;
  {  // request
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->request ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "request", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // response
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->response ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "response", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // report
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->report ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "report", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // print
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->print ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "print", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // jetson
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->jetson ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "jetson", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l476
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->l476 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l476", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // f103
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->f103 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "f103", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // comm_jetson
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->comm_jetson.data,
      strlen(ros_message->comm_jetson.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "comm_jetson", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // comm_l476
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->comm_l476.data,
      strlen(ros_message->comm_l476.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "comm_l476", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // comm_f103
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->comm_f103.data,
      strlen(ros_message->comm_f103.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "comm_f103", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->battery.data,
      strlen(ros_message->battery.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ultrasonics
    PyObject * field = NULL;
    size_t size = 6;
    rosidl_runtime_c__String * src = ros_message->ultrasonics;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      PyObject * decoded_item = PyUnicode_DecodeUTF8(src[i].data, strlen(src[i].data), "strict");
      if (!decoded_item) {
        return NULL;
      }
      int rc = PyList_SetItem(field, i, decoded_item);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "ultrasonics", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->gps.data,
      strlen(ros_message->gps.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->imu.data,
      strlen(ros_message->imu.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lidar
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->lidar.data,
      strlen(ros_message->lidar.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "lidar", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // camera
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->camera.data,
      strlen(ros_message->camera.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "camera", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
