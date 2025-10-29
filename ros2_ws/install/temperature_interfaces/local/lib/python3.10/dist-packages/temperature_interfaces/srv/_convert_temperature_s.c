// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from temperature_interfaces:srv/ConvertTemperature.idl
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
#include "temperature_interfaces/srv/detail/convert_temperature__struct.h"
#include "temperature_interfaces/srv/detail/convert_temperature__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool temperature_interfaces__srv__convert_temperature__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[75];
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
    assert(strncmp("temperature_interfaces.srv._convert_temperature.ConvertTemperature_Request", full_classname_dest, 74) == 0);
  }
  temperature_interfaces__srv__ConvertTemperature_Request * ros_message = _ros_message;
  {  // input_temp
    PyObject * field = PyObject_GetAttrString(_pymsg, "input_temp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->input_temp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // conversion_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "conversion_type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->conversion_type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * temperature_interfaces__srv__convert_temperature__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ConvertTemperature_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("temperature_interfaces.srv._convert_temperature");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ConvertTemperature_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  temperature_interfaces__srv__ConvertTemperature_Request * ros_message = (temperature_interfaces__srv__ConvertTemperature_Request *)raw_ros_message;
  {  // input_temp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->input_temp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "input_temp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // conversion_type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->conversion_type.data,
      strlen(ros_message->conversion_type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "conversion_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "temperature_interfaces/srv/detail/convert_temperature__struct.h"
// already included above
// #include "temperature_interfaces/srv/detail/convert_temperature__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool temperature_interfaces__srv__convert_temperature__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[76];
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
    assert(strncmp("temperature_interfaces.srv._convert_temperature.ConvertTemperature_Response", full_classname_dest, 75) == 0);
  }
  temperature_interfaces__srv__ConvertTemperature_Response * ros_message = _ros_message;
  {  // converted_temp
    PyObject * field = PyObject_GetAttrString(_pymsg, "converted_temp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->converted_temp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * temperature_interfaces__srv__convert_temperature__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ConvertTemperature_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("temperature_interfaces.srv._convert_temperature");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ConvertTemperature_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  temperature_interfaces__srv__ConvertTemperature_Response * ros_message = (temperature_interfaces__srv__ConvertTemperature_Response *)raw_ros_message;
  {  // converted_temp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->converted_temp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "converted_temp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
