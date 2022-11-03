# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:msg/GeneralData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GeneralData(type):
    """Metaclass of message 'GeneralData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces.msg.GeneralData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__general_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__general_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__general_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__general_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__general_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GeneralData(metaclass=Metaclass_GeneralData):
    """Message class 'GeneralData'."""

    __slots__ = [
        '_battery_level',
        '_temperature',
        '_pressure',
        '_humidity',
    ]

    _fields_and_field_types = {
        'battery_level': 'float',
        'temperature': 'float',
        'pressure': 'int16',
        'humidity': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.battery_level = kwargs.get('battery_level', float())
        self.temperature = kwargs.get('temperature', float())
        self.pressure = kwargs.get('pressure', int())
        self.humidity = kwargs.get('humidity', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.battery_level != other.battery_level:
            return False
        if self.temperature != other.temperature:
            return False
        if self.pressure != other.pressure:
            return False
        if self.humidity != other.humidity:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def battery_level(self):
        """Message field 'battery_level'."""
        return self._battery_level

    @battery_level.setter
    def battery_level(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_level' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'battery_level' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._battery_level = value

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temperature' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'temperature' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._temperature = value

    @builtins.property
    def pressure(self):
        """Message field 'pressure'."""
        return self._pressure

    @pressure.setter
    def pressure(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pressure' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'pressure' field must be an integer in [-32768, 32767]"
        self._pressure = value

    @builtins.property
    def humidity(self):
        """Message field 'humidity'."""
        return self._humidity

    @humidity.setter
    def humidity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'humidity' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'humidity' field must be an integer in [-128, 127]"
        self._humidity = value
