# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:msg/SteeringCalibration.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SteeringCalibration(type):
    """Metaclass of message 'SteeringCalibration'."""

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
                'interfaces.msg.SteeringCalibration')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__steering_calibration
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__steering_calibration
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__steering_calibration
            cls._TYPE_SUPPORT = module.type_support_msg__msg__steering_calibration
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__steering_calibration

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'REQUEST__DEFAULT': False,
            'IN_PROGRESS__DEFAULT': False,
            'USER_NEED__DEFAULT': False,
            'STATUS__DEFAULT': 0,
        }

    @property
    def REQUEST__DEFAULT(cls):
        """Return default value for message field 'request'."""
        return False

    @property
    def IN_PROGRESS__DEFAULT(cls):
        """Return default value for message field 'in_progress'."""
        return False

    @property
    def USER_NEED__DEFAULT(cls):
        """Return default value for message field 'user_need'."""
        return False

    @property
    def STATUS__DEFAULT(cls):
        """Return default value for message field 'status'."""
        return 0


class SteeringCalibration(metaclass=Metaclass_SteeringCalibration):
    """Message class 'SteeringCalibration'."""

    __slots__ = [
        '_request',
        '_in_progress',
        '_user_need',
        '_status',
    ]

    _fields_and_field_types = {
        'request': 'boolean',
        'in_progress': 'boolean',
        'user_need': 'boolean',
        'status': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.request = kwargs.get(
            'request', SteeringCalibration.REQUEST__DEFAULT)
        self.in_progress = kwargs.get(
            'in_progress', SteeringCalibration.IN_PROGRESS__DEFAULT)
        self.user_need = kwargs.get(
            'user_need', SteeringCalibration.USER_NEED__DEFAULT)
        self.status = kwargs.get(
            'status', SteeringCalibration.STATUS__DEFAULT)

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
        if self.request != other.request:
            return False
        if self.in_progress != other.in_progress:
            return False
        if self.user_need != other.user_need:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'request' field must be of type 'bool'"
        self._request = value

    @builtins.property
    def in_progress(self):
        """Message field 'in_progress'."""
        return self._in_progress

    @in_progress.setter
    def in_progress(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'in_progress' field must be of type 'bool'"
        self._in_progress = value

    @builtins.property
    def user_need(self):
        """Message field 'user_need'."""
        return self._user_need

    @user_need.setter
    def user_need(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'user_need' field must be of type 'bool'"
        self._user_need = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value
