# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from msgs/HomeAndLimitRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class HomeAndLimitRequest(genpy.Message):
  _md5sum = "3495607945e38c175ee34dd54966e097"
  _type = "msgs/HomeAndLimitRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64[3] home_xyz
float64[4] home_oreintation
float64[6] limit_range
"""
  __slots__ = ['home_xyz','home_oreintation','limit_range']
  _slot_types = ['float64[3]','float64[4]','float64[6]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       home_xyz,home_oreintation,limit_range

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HomeAndLimitRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.home_xyz is None:
        self.home_xyz = [0.] * 3
      if self.home_oreintation is None:
        self.home_oreintation = [0.] * 4
      if self.limit_range is None:
        self.limit_range = [0.] * 6
    else:
      self.home_xyz = [0.] * 3
      self.home_oreintation = [0.] * 4
      self.limit_range = [0.] * 6

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
      buff.write(_get_struct_3d().pack(*self.home_xyz))
      buff.write(_get_struct_4d().pack(*self.home_oreintation))
      buff.write(_get_struct_6d().pack(*self.limit_range))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 24
      self.home_xyz = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 32
      self.home_oreintation = _get_struct_4d().unpack(str[start:end])
      start = end
      end += 48
      self.limit_range = _get_struct_6d().unpack(str[start:end])
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
      buff.write(self.home_xyz.tostring())
      buff.write(self.home_oreintation.tostring())
      buff.write(self.limit_range.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 24
      self.home_xyz = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 32
      self.home_oreintation = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=4)
      start = end
      end += 48
      self.limit_range = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=6)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from msgs/HomeAndLimitResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class HomeAndLimitResponse(genpy.Message):
  _md5sum = "a265a3e43e23769318ae493072a49313"
  _type = "msgs/HomeAndLimitResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 done
"""
  __slots__ = ['done']
  _slot_types = ['int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       done

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HomeAndLimitResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.done is None:
        self.done = 0
    else:
      self.done = 0

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
      buff.write(_get_struct_b().pack(self.done))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.done,) = _get_struct_b().unpack(str[start:end])
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
      buff.write(_get_struct_b().pack(self.done))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.done,) = _get_struct_b().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_b = None
def _get_struct_b():
    global _struct_b
    if _struct_b is None:
        _struct_b = struct.Struct("<b")
    return _struct_b
class HomeAndLimit(object):
  _type          = 'msgs/HomeAndLimit'
  _md5sum = '37986a4fb94d2bd1e620d61c5fcb7476'
  _request_class  = HomeAndLimitRequest
  _response_class = HomeAndLimitResponse
