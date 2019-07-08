# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from estimation/IRMarker.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class IRMarker(genpy.Message):
  _md5sum = "74186740d14e922a7e5ca083a6795f31"
  _type = "estimation/IRMarker"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ROS Message that contains pixel-space IR marker locations and labels.

# ID of the landmark (e.g. gate) that the IR marker is attached to
std_msgs/String landmarkID
# ID of individual marker.
std_msgs/String markerID

float32 x
float32 y
# Z is the distance from the camera in meters.
float32 z
================================================================================
MSG: std_msgs/String
string data
"""
  __slots__ = ['landmarkID','markerID','x','y','z']
  _slot_types = ['std_msgs/String','std_msgs/String','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       landmarkID,markerID,x,y,z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(IRMarker, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.landmarkID is None:
        self.landmarkID = std_msgs.msg.String()
      if self.markerID is None:
        self.markerID = std_msgs.msg.String()
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
    else:
      self.landmarkID = std_msgs.msg.String()
      self.markerID = std_msgs.msg.String()
      self.x = 0.
      self.y = 0.
      self.z = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.landmarkID.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.markerID.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.landmarkID is None:
        self.landmarkID = std_msgs.msg.String()
      if self.markerID is None:
        self.markerID = std_msgs.msg.String()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.landmarkID.data = str[start:end].decode('utf-8')
      else:
        self.landmarkID.data = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.markerID.data = str[start:end].decode('utf-8')
      else:
        self.markerID.data = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.landmarkID.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.markerID.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.landmarkID is None:
        self.landmarkID = std_msgs.msg.String()
      if self.markerID is None:
        self.markerID = std_msgs.msg.String()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.landmarkID.data = str[start:end].decode('utf-8')
      else:
        self.landmarkID.data = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.markerID.data = str[start:end].decode('utf-8')
      else:
        self.markerID.data = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
