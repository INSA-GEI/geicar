# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:msg/Gnss.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Gnss(type):
    """Metaclass of message 'Gnss'."""

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
                'interfaces.msg.Gnss')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gnss
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gnss
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gnss
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gnss
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gnss

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'LATITUDE__DEFAULT': -1.0,
            'LONGITUDE__DEFAULT': -1.0,
            'ALTITUDE__DEFAULT': -1.0,
            'QUALITY__DEFAULT': -1,
            'HACC__DEFAULT': -1.0,
            'VACC__DEFAULT': -1.0,
        }

    @property
    def LATITUDE__DEFAULT(cls):
        """Return default value for message field 'latitude'."""
        return -1.0

    @property
    def LONGITUDE__DEFAULT(cls):
        """Return default value for message field 'longitude'."""
        return -1.0

    @property
    def ALTITUDE__DEFAULT(cls):
        """Return default value for message field 'altitude'."""
        return -1.0

    @property
    def QUALITY__DEFAULT(cls):
        """Return default value for message field 'quality'."""
        return -1

    @property
    def HACC__DEFAULT(cls):
        """Return default value for message field 'hacc'."""
        return -1.0

    @property
    def VACC__DEFAULT(cls):
        """Return default value for message field 'vacc'."""
        return -1.0


class Gnss(metaclass=Metaclass_Gnss):
    """Message class 'Gnss'."""

    __slots__ = [
        '_latitude',
        '_longitude',
        '_altitude',
        '_quality',
        '_hacc',
        '_vacc',
    ]

    _fields_and_field_types = {
        'latitude': 'double',
        'longitude': 'double',
        'altitude': 'double',
        'quality': 'int8',
        'hacc': 'double',
        'vacc': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.latitude = kwargs.get(
            'latitude', Gnss.LATITUDE__DEFAULT)
        self.longitude = kwargs.get(
            'longitude', Gnss.LONGITUDE__DEFAULT)
        self.altitude = kwargs.get(
            'altitude', Gnss.ALTITUDE__DEFAULT)
        self.quality = kwargs.get(
            'quality', Gnss.QUALITY__DEFAULT)
        self.hacc = kwargs.get(
            'hacc', Gnss.HACC__DEFAULT)
        self.vacc = kwargs.get(
            'vacc', Gnss.VACC__DEFAULT)

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
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.altitude != other.altitude:
            return False
        if self.quality != other.quality:
            return False
        if self.hacc != other.hacc:
            return False
        if self.vacc != other.vacc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert value >= -1.7976931348623157e+308 and value <= 1.7976931348623157e+308, \
                "The 'latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._latitude = value

    @builtins.property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert value >= -1.7976931348623157e+308 and value <= 1.7976931348623157e+308, \
                "The 'longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._longitude = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert value >= -1.7976931348623157e+308 and value <= 1.7976931348623157e+308, \
                "The 'altitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._altitude = value

    @builtins.property
    def quality(self):
        """Message field 'quality'."""
        return self._quality

    @quality.setter
    def quality(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'quality' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'quality' field must be an integer in [-128, 127]"
        self._quality = value

    @builtins.property
    def hacc(self):
        """Message field 'hacc'."""
        return self._hacc

    @hacc.setter
    def hacc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'hacc' field must be of type 'float'"
            assert value >= -1.7976931348623157e+308 and value <= 1.7976931348623157e+308, \
                "The 'hacc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._hacc = value

    @builtins.property
    def vacc(self):
        """Message field 'vacc'."""
        return self._vacc

    @vacc.setter
    def vacc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vacc' field must be of type 'float'"
            assert value >= -1.7976931348623157e+308 and value <= 1.7976931348623157e+308, \
                "The 'vacc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vacc = value
