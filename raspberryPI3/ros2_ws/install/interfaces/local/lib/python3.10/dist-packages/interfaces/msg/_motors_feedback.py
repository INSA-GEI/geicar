# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:msg/MotorsFeedback.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorsFeedback(type):
    """Metaclass of message 'MotorsFeedback'."""

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
                'interfaces.msg.MotorsFeedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motors_feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motors_feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motors_feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motors_feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motors_feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'LEFT_REAR_ODOMETRY__DEFAULT': 0,
            'RIGHT_REAR_ODOMETRY__DEFAULT': 0,
            'LEFT_REAR_SPEED__DEFAULT': 0.0,
            'RIGHT_REAR_SPEED__DEFAULT': 0.0,
            'STEERING_ANGLE__DEFAULT': 0.0,
        }

    @property
    def LEFT_REAR_ODOMETRY__DEFAULT(cls):
        """Return default value for message field 'left_rear_odometry'."""
        return 0

    @property
    def RIGHT_REAR_ODOMETRY__DEFAULT(cls):
        """Return default value for message field 'right_rear_odometry'."""
        return 0

    @property
    def LEFT_REAR_SPEED__DEFAULT(cls):
        """Return default value for message field 'left_rear_speed'."""
        return 0.0

    @property
    def RIGHT_REAR_SPEED__DEFAULT(cls):
        """Return default value for message field 'right_rear_speed'."""
        return 0.0

    @property
    def STEERING_ANGLE__DEFAULT(cls):
        """Return default value for message field 'steering_angle'."""
        return 0.0


class MotorsFeedback(metaclass=Metaclass_MotorsFeedback):
    """Message class 'MotorsFeedback'."""

    __slots__ = [
        '_left_rear_odometry',
        '_right_rear_odometry',
        '_left_rear_speed',
        '_right_rear_speed',
        '_steering_angle',
    ]

    _fields_and_field_types = {
        'left_rear_odometry': 'int8',
        'right_rear_odometry': 'int8',
        'left_rear_speed': 'float',
        'right_rear_speed': 'float',
        'steering_angle': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.left_rear_odometry = kwargs.get(
            'left_rear_odometry', MotorsFeedback.LEFT_REAR_ODOMETRY__DEFAULT)
        self.right_rear_odometry = kwargs.get(
            'right_rear_odometry', MotorsFeedback.RIGHT_REAR_ODOMETRY__DEFAULT)
        self.left_rear_speed = kwargs.get(
            'left_rear_speed', MotorsFeedback.LEFT_REAR_SPEED__DEFAULT)
        self.right_rear_speed = kwargs.get(
            'right_rear_speed', MotorsFeedback.RIGHT_REAR_SPEED__DEFAULT)
        self.steering_angle = kwargs.get(
            'steering_angle', MotorsFeedback.STEERING_ANGLE__DEFAULT)

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
        if self.left_rear_odometry != other.left_rear_odometry:
            return False
        if self.right_rear_odometry != other.right_rear_odometry:
            return False
        if self.left_rear_speed != other.left_rear_speed:
            return False
        if self.right_rear_speed != other.right_rear_speed:
            return False
        if self.steering_angle != other.steering_angle:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def left_rear_odometry(self):
        """Message field 'left_rear_odometry'."""
        return self._left_rear_odometry

    @left_rear_odometry.setter
    def left_rear_odometry(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_rear_odometry' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'left_rear_odometry' field must be an integer in [-128, 127]"
        self._left_rear_odometry = value

    @builtins.property
    def right_rear_odometry(self):
        """Message field 'right_rear_odometry'."""
        return self._right_rear_odometry

    @right_rear_odometry.setter
    def right_rear_odometry(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_rear_odometry' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'right_rear_odometry' field must be an integer in [-128, 127]"
        self._right_rear_odometry = value

    @builtins.property
    def left_rear_speed(self):
        """Message field 'left_rear_speed'."""
        return self._left_rear_speed

    @left_rear_speed.setter
    def left_rear_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_rear_speed' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'left_rear_speed' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._left_rear_speed = value

    @builtins.property
    def right_rear_speed(self):
        """Message field 'right_rear_speed'."""
        return self._right_rear_speed

    @right_rear_speed.setter
    def right_rear_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_rear_speed' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'right_rear_speed' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._right_rear_speed = value

    @builtins.property
    def steering_angle(self):
        """Message field 'steering_angle'."""
        return self._steering_angle

    @steering_angle.setter
    def steering_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steering_angle' field must be of type 'float'"
            assert value >= -3.402823e+38 and value <= 3.402823e+38, \
                "The 'steering_angle' field must be a float in [-3.402823e+38, 3.402823e+38]"
        self._steering_angle = value
