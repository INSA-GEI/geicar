# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:msg/SystemCheck.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SystemCheck(type):
    """Metaclass of message 'SystemCheck'."""

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
                'interfaces.msg.SystemCheck')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__system_check
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__system_check
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__system_check
            cls._TYPE_SUPPORT = module.type_support_msg__msg__system_check
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__system_check

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'REQUEST__DEFAULT': False,
            'RESPONSE__DEFAULT': False,
            'REPORT__DEFAULT': False,
            'PRINT__DEFAULT': False,
            'JETSON__DEFAULT': False,
            'L476__DEFAULT': False,
            'F103__DEFAULT': False,
        }

    @property
    def REQUEST__DEFAULT(cls):
        """Return default value for message field 'request'."""
        return False

    @property
    def RESPONSE__DEFAULT(cls):
        """Return default value for message field 'response'."""
        return False

    @property
    def REPORT__DEFAULT(cls):
        """Return default value for message field 'report'."""
        return False

    @property
    def PRINT__DEFAULT(cls):
        """Return default value for message field 'print'."""
        return False

    @property
    def JETSON__DEFAULT(cls):
        """Return default value for message field 'jetson'."""
        return False

    @property
    def L476__DEFAULT(cls):
        """Return default value for message field 'l476'."""
        return False

    @property
    def F103__DEFAULT(cls):
        """Return default value for message field 'f103'."""
        return False


class SystemCheck(metaclass=Metaclass_SystemCheck):
    """Message class 'SystemCheck'."""

    __slots__ = [
        '_request',
        '_response',
        '_report',
        '_print',
        '_jetson',
        '_l476',
        '_f103',
        '_comm_jetson',
        '_comm_l476',
        '_comm_f103',
        '_battery',
        '_ultrasonics',
        '_gps',
        '_imu',
        '_lidar',
        '_camera',
    ]

    _fields_and_field_types = {
        'request': 'boolean',
        'response': 'boolean',
        'report': 'boolean',
        'print': 'boolean',
        'jetson': 'boolean',
        'l476': 'boolean',
        'f103': 'boolean',
        'comm_jetson': 'string',
        'comm_l476': 'string',
        'comm_f103': 'string',
        'battery': 'string',
        'ultrasonics': 'string[6]',
        'gps': 'string',
        'imu': 'string',
        'lidar': 'string',
        'camera': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.UnboundedString(), 6),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.request = kwargs.get(
            'request', SystemCheck.REQUEST__DEFAULT)
        self.response = kwargs.get(
            'response', SystemCheck.RESPONSE__DEFAULT)
        self.report = kwargs.get(
            'report', SystemCheck.REPORT__DEFAULT)
        self.print = kwargs.get(
            'print', SystemCheck.PRINT__DEFAULT)
        self.jetson = kwargs.get(
            'jetson', SystemCheck.JETSON__DEFAULT)
        self.l476 = kwargs.get(
            'l476', SystemCheck.L476__DEFAULT)
        self.f103 = kwargs.get(
            'f103', SystemCheck.F103__DEFAULT)
        self.comm_jetson = kwargs.get('comm_jetson', str())
        self.comm_l476 = kwargs.get('comm_l476', str())
        self.comm_f103 = kwargs.get('comm_f103', str())
        self.battery = kwargs.get('battery', str())
        self.ultrasonics = kwargs.get(
            'ultrasonics',
            [str() for x in range(6)]
        )
        self.gps = kwargs.get('gps', str())
        self.imu = kwargs.get('imu', str())
        self.lidar = kwargs.get('lidar', str())
        self.camera = kwargs.get('camera', str())

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
        if self.response != other.response:
            return False
        if self.report != other.report:
            return False
        if self.print != other.print:
            return False
        if self.jetson != other.jetson:
            return False
        if self.l476 != other.l476:
            return False
        if self.f103 != other.f103:
            return False
        if self.comm_jetson != other.comm_jetson:
            return False
        if self.comm_l476 != other.comm_l476:
            return False
        if self.comm_f103 != other.comm_f103:
            return False
        if self.battery != other.battery:
            return False
        if self.ultrasonics != other.ultrasonics:
            return False
        if self.gps != other.gps:
            return False
        if self.imu != other.imu:
            return False
        if self.lidar != other.lidar:
            return False
        if self.camera != other.camera:
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
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'response' field must be of type 'bool'"
        self._response = value

    @builtins.property
    def report(self):
        """Message field 'report'."""
        return self._report

    @report.setter
    def report(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'report' field must be of type 'bool'"
        self._report = value

    @builtins.property  # noqa: A003
    def print(self):  # noqa: A003
        """Message field 'print'."""
        return self._print

    @print.setter  # noqa: A003
    def print(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'print' field must be of type 'bool'"
        self._print = value

    @builtins.property
    def jetson(self):
        """Message field 'jetson'."""
        return self._jetson

    @jetson.setter
    def jetson(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'jetson' field must be of type 'bool'"
        self._jetson = value

    @builtins.property
    def l476(self):
        """Message field 'l476'."""
        return self._l476

    @l476.setter
    def l476(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'l476' field must be of type 'bool'"
        self._l476 = value

    @builtins.property
    def f103(self):
        """Message field 'f103'."""
        return self._f103

    @f103.setter
    def f103(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'f103' field must be of type 'bool'"
        self._f103 = value

    @builtins.property
    def comm_jetson(self):
        """Message field 'comm_jetson'."""
        return self._comm_jetson

    @comm_jetson.setter
    def comm_jetson(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'comm_jetson' field must be of type 'str'"
        self._comm_jetson = value

    @builtins.property
    def comm_l476(self):
        """Message field 'comm_l476'."""
        return self._comm_l476

    @comm_l476.setter
    def comm_l476(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'comm_l476' field must be of type 'str'"
        self._comm_l476 = value

    @builtins.property
    def comm_f103(self):
        """Message field 'comm_f103'."""
        return self._comm_f103

    @comm_f103.setter
    def comm_f103(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'comm_f103' field must be of type 'str'"
        self._comm_f103 = value

    @builtins.property
    def battery(self):
        """Message field 'battery'."""
        return self._battery

    @battery.setter
    def battery(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'battery' field must be of type 'str'"
        self._battery = value

    @builtins.property
    def ultrasonics(self):
        """Message field 'ultrasonics'."""
        return self._ultrasonics

    @ultrasonics.setter
    def ultrasonics(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 6 and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'ultrasonics' field must be a set or sequence with length 6 and each value of type 'str'"
        self._ultrasonics = value

    @builtins.property
    def gps(self):
        """Message field 'gps'."""
        return self._gps

    @gps.setter
    def gps(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'gps' field must be of type 'str'"
        self._gps = value

    @builtins.property
    def imu(self):
        """Message field 'imu'."""
        return self._imu

    @imu.setter
    def imu(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'imu' field must be of type 'str'"
        self._imu = value

    @builtins.property
    def lidar(self):
        """Message field 'lidar'."""
        return self._lidar

    @lidar.setter
    def lidar(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'lidar' field must be of type 'str'"
        self._lidar = value

    @builtins.property
    def camera(self):
        """Message field 'camera'."""
        return self._camera

    @camera.setter
    def camera(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'camera' field must be of type 'str'"
        self._camera = value
