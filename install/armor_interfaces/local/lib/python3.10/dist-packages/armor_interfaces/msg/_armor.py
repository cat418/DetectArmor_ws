# generated from rosidl_generator_py/resource/_idl.py.em
# with input from armor_interfaces:msg/Armor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Armor(type):
    """Metaclass of message 'Armor'."""

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
            module = import_type_support('armor_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'armor_interfaces.msg.Armor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__armor
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__armor
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__armor
            cls._TYPE_SUPPORT = module.type_support_msg__msg__armor
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__armor

            from geometry_msgs.msg import Point32
            if Point32.__class__._TYPE_SUPPORT is None:
                Point32.__class__.__import_type_support__()

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Armor(metaclass=Metaclass_Armor):
    """Message class 'Armor'."""

    __slots__ = [
        '_number',
        '_type',
        '_color',
        '_distance_to_center',
        '_apexs',
        '_pose',
        '_world_pose',
    ]

    _fields_and_field_types = {
        'number': 'uint8',
        'type': 'string',
        'color': 'string',
        'distance_to_center': 'float',
        'apexs': 'geometry_msgs/Point32[4]',
        'pose': 'geometry_msgs/Pose',
        'world_pose': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point32'), 4),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.number = kwargs.get('number', int())
        self.type = kwargs.get('type', str())
        self.color = kwargs.get('color', str())
        self.distance_to_center = kwargs.get('distance_to_center', float())
        from geometry_msgs.msg import Point32
        self.apexs = kwargs.get(
            'apexs',
            [Point32() for x in range(4)]
        )
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())
        from geometry_msgs.msg import Pose
        self.world_pose = kwargs.get('world_pose', Pose())

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
        if self.number != other.number:
            return False
        if self.type != other.type:
            return False
        if self.color != other.color:
            return False
        if self.distance_to_center != other.distance_to_center:
            return False
        if self.apexs != other.apexs:
            return False
        if self.pose != other.pose:
            return False
        if self.world_pose != other.world_pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def number(self):
        """Message field 'number'."""
        return self._number

    @number.setter
    def number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'number' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'number' field must be an unsigned integer in [0, 255]"
        self._number = value

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'type' field must be of type 'str'"
        self._type = value

    @builtins.property
    def color(self):
        """Message field 'color'."""
        return self._color

    @color.setter
    def color(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'color' field must be of type 'str'"
        self._color = value

    @builtins.property
    def distance_to_center(self):
        """Message field 'distance_to_center'."""
        return self._distance_to_center

    @distance_to_center.setter
    def distance_to_center(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_to_center' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance_to_center' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance_to_center = value

    @builtins.property
    def apexs(self):
        """Message field 'apexs'."""
        return self._apexs

    @apexs.setter
    def apexs(self, value):
        if __debug__:
            from geometry_msgs.msg import Point32
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
                 len(value) == 4 and
                 all(isinstance(v, Point32) for v in value) and
                 True), \
                "The 'apexs' field must be a set or sequence with length 4 and each value of type 'Point32'"
        self._apexs = value

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'pose' field must be a sub message of type 'Pose'"
        self._pose = value

    @builtins.property
    def world_pose(self):
        """Message field 'world_pose'."""
        return self._world_pose

    @world_pose.setter
    def world_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'world_pose' field must be a sub message of type 'Pose'"
        self._world_pose = value
