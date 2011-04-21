"""autogenerated by genmsg_py from move_headResult.msg. Do not edit."""
import roslib.message
import struct

import geometry_msgs.msg

class move_headResult(roslib.message.Message):
  _md5sum = "4e66d656664733a19f712b5693e6334b"
  _type = "jido_move_head/move_headResult"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#result definition
geometry_msgs/Vector3 resultPanTilt

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['resultPanTilt']
  _slot_types = ['geometry_msgs/Vector3']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       resultPanTilt
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(move_headResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.resultPanTilt is None:
        self.resultPanTilt = geometry_msgs.msg.Vector3()
    else:
      self.resultPanTilt = geometry_msgs.msg.Vector3()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3d.pack(_x.resultPanTilt.x, _x.resultPanTilt.y, _x.resultPanTilt.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.resultPanTilt is None:
        self.resultPanTilt = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.resultPanTilt.x, _x.resultPanTilt.y, _x.resultPanTilt.z,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3d.pack(_x.resultPanTilt.x, _x.resultPanTilt.y, _x.resultPanTilt.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.resultPanTilt is None:
        self.resultPanTilt = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.resultPanTilt.x, _x.resultPanTilt.y, _x.resultPanTilt.z,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3d = struct.Struct("<3d")
