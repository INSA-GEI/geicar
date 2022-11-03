# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:msg/MotorsOrder.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorsOrder(type):
    """Metaclass of message 'MotorsOrder'."""

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
                'interfaces.msg.MotorsOrder')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motors_order
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motors_order
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motors_order
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motors_order
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motors_order

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'RIGHT_REAR_PWM__DEFAULT': 50,
            'LEFT_REAR_PWM__DEFAULT': 50,
            'STEERING_PWM__DEFAULT': 50,
        }

    @property
    def RIGHT_REAR_PWM__DEFAULT(cls):
        """Return default value for message field 'right_rear_pwm'."""
        return 50

    @property
    def LEFT_REAR_PWM__DEFAULT(cls):
        """Return default value for message field 'left_rear_pwm'."""
        return 50

    @property
    def STEERING_PWM__DEFAULT(cls):
        """Return default value for message field 'steering_pwm'."""
        return 50


class MotorsOrder(metaclass=Metaclass_MotorsOrder):
    """Message class 'MotorsOrder'."""

    __slots__ = [
        '_right_rear_pwm',
        '_left_rear_pwm',
        '_steering_pwm',
    ]

    _fields_and_field_types = {
        'right_rear_pwm': 'int8',
        'left_rear_pwm': 'int8',
        'steering_pwm': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.right_rear_pwm = kwargs.get(
            'right_rear_pwm', MotorsOrder.RIGHT_REAR_PWM__DEFAULT)
        self.left_rear_pwm = kwargs.get(
            'left_rear_pwm', MotorsOrder.LEFT_REAR_PWM__DEFAULT)
        self.steering_pwm = kwargs.get(
            'steering_pwm', MotorsOrder.STEERING_PWM__DEFAULT)

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
        if self.right_rear_pwm != other.right_rear_pwm:
            return False
        if self.left_rear_pwm != other.left_rear_pwm:
            return False
        if self.steering_pwm != other.steering_pwm:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def right_rear_pwm(self):
        """Message field 'right_rear_pwm'."""
        return self._right_rear_pwm

    @right_rear_pwm.setter
    def right_rear_pwm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_rear_pwm' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'right_rear_pwm' field must be an integer in [-128, 127]"
        self._right_rear_pwm = value

    @builtins.property
    def left_rear_pwm(self):
        """Message field 'left_rear_pwm'."""
        return self._left_rear_pwm

    @left_rear_pwm.setter
    def left_rear_pwm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_rear_pwm' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'left_rear_pwm' field must be an integer in [-128, 127]"
        self._left_rear_pwm = value

    @builtins.property
    def steering_pwm(self):
        """Message field 'steering_pwm'."""
        return self._steering_pwm

    @steering_pwm.setter
    def steering_pwm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'steering_pwm' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'steering_pwm' field must be an integer in [-128, 127]"
        self._steering_pwm = value
