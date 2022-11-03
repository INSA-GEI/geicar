# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:msg/JoystickOrder.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_JoystickOrder(type):
    """Metaclass of message 'JoystickOrder'."""

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
                'interfaces.msg.JoystickOrder')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__joystick_order
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__joystick_order
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__joystick_order
            cls._TYPE_SUPPORT = module.type_support_msg__msg__joystick_order
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__joystick_order

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'START__DEFAULT': False,
            'MODE__DEFAULT': 0,
            'THROTTLE__DEFAULT': 0.0,
            'STEER__DEFAULT': 0.0,
            'REVERSE__DEFAULT': False,
        }

    @property
    def START__DEFAULT(cls):
        """Return default value for message field 'start'."""
        return False

    @property
    def MODE__DEFAULT(cls):
        """Return default value for message field 'mode'."""
        return 0

    @property
    def THROTTLE__DEFAULT(cls):
        """Return default value for message field 'throttle'."""
        return 0.0

    @property
    def STEER__DEFAULT(cls):
        """Return default value for message field 'steer'."""
        return 0.0

    @property
    def REVERSE__DEFAULT(cls):
        """Return default value for message field 'reverse'."""
        return False


class JoystickOrder(metaclass=Metaclass_JoystickOrder):
    """Message class 'JoystickOrder'."""

    __slots__ = [
        '_start',
        '_mode',
        '_throttle',
        '_steer',
        '_reverse',
    ]

    _fields_and_field_types = {
        'start': 'boolean',
        'mode': 'int8',
        'throttle': 'float',
        'steer': 'float',
        'reverse': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.start = kwargs.get(
            'start', JoystickOrder.START__DEFAULT)
        self.mode = kwargs.get(
            'mode', JoystickOrder.MODE__DEFAULT)
        self.throttle = kwargs.get(
            'throttle', JoystickOrder.THROTTLE__DEFAULT)
        self.steer = kwargs.get(
            'steer', JoystickOrder.STEER__DEFAULT)
        self.reverse = kwargs.get(
            'reverse', JoystickOrder.REVERSE__DEFAULT)

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
        if self.start != other.start:
            return False
        if self.mode != other.mode:
            return False
        if self.throttle != other.throttle:
            return False
        if self.steer != other.steer:
            return False
        if self.reverse != other.reverse:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def start(self):
        """Message field 'start'."""
        return self._start

    @start.setter
    def start(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'start' field must be of type 'bool'"
        self._start = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'mode' field must be an integer in [-128, 127]"
        self._mode = value

    @builtins.property
    def throttle(self):
        """Message field 'throttle'."""
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'throttle' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'throttle' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._throttle = value

    @builtins.property
    def steer(self):
        """Message field 'steer'."""
        return self._steer

    @steer.setter
    def steer(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steer' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'steer' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._steer = value

    @builtins.property
    def reverse(self):
        """Message field 'reverse'."""
        return self._reverse

    @reverse.setter
    def reverse(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'reverse' field must be of type 'bool'"
        self._reverse = value
