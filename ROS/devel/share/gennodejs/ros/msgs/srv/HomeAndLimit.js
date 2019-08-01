// Auto-generated. Do not edit!

// (in-package msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class HomeAndLimitRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.home_xyz = null;
      this.home_oreintation = null;
      this.limit_range = null;
    }
    else {
      if (initObj.hasOwnProperty('home_xyz')) {
        this.home_xyz = initObj.home_xyz
      }
      else {
        this.home_xyz = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('home_oreintation')) {
        this.home_oreintation = initObj.home_oreintation
      }
      else {
        this.home_oreintation = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('limit_range')) {
        this.limit_range = initObj.limit_range
      }
      else {
        this.limit_range = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HomeAndLimitRequest
    // Check that the constant length array field [home_xyz] has the right length
    if (obj.home_xyz.length !== 3) {
      throw new Error('Unable to serialize array field home_xyz - length must be 3')
    }
    // Serialize message field [home_xyz]
    bufferOffset = _arraySerializer.float64(obj.home_xyz, buffer, bufferOffset, 3);
    // Check that the constant length array field [home_oreintation] has the right length
    if (obj.home_oreintation.length !== 4) {
      throw new Error('Unable to serialize array field home_oreintation - length must be 4')
    }
    // Serialize message field [home_oreintation]
    bufferOffset = _arraySerializer.float64(obj.home_oreintation, buffer, bufferOffset, 4);
    // Check that the constant length array field [limit_range] has the right length
    if (obj.limit_range.length !== 6) {
      throw new Error('Unable to serialize array field limit_range - length must be 6')
    }
    // Serialize message field [limit_range]
    bufferOffset = _arraySerializer.float64(obj.limit_range, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HomeAndLimitRequest
    let len;
    let data = new HomeAndLimitRequest(null);
    // Deserialize message field [home_xyz]
    data.home_xyz = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [home_oreintation]
    data.home_oreintation = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [limit_range]
    data.limit_range = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 104;
  }

  static datatype() {
    // Returns string type for a service object
    return 'msgs/HomeAndLimitRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3495607945e38c175ee34dd54966e097';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[3] home_xyz
    float64[4] home_oreintation
    float64[6] limit_range
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HomeAndLimitRequest(null);
    if (msg.home_xyz !== undefined) {
      resolved.home_xyz = msg.home_xyz;
    }
    else {
      resolved.home_xyz = new Array(3).fill(0)
    }

    if (msg.home_oreintation !== undefined) {
      resolved.home_oreintation = msg.home_oreintation;
    }
    else {
      resolved.home_oreintation = new Array(4).fill(0)
    }

    if (msg.limit_range !== undefined) {
      resolved.limit_range = msg.limit_range;
    }
    else {
      resolved.limit_range = new Array(6).fill(0)
    }

    return resolved;
    }
};

class HomeAndLimitResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.done = null;
    }
    else {
      if (initObj.hasOwnProperty('done')) {
        this.done = initObj.done
      }
      else {
        this.done = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HomeAndLimitResponse
    // Serialize message field [done]
    bufferOffset = _serializer.int8(obj.done, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HomeAndLimitResponse
    let len;
    let data = new HomeAndLimitResponse(null);
    // Deserialize message field [done]
    data.done = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'msgs/HomeAndLimitResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a265a3e43e23769318ae493072a49313';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 done
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HomeAndLimitResponse(null);
    if (msg.done !== undefined) {
      resolved.done = msg.done;
    }
    else {
      resolved.done = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: HomeAndLimitRequest,
  Response: HomeAndLimitResponse,
  md5sum() { return '37986a4fb94d2bd1e620d61c5fcb7476'; },
  datatype() { return 'msgs/HomeAndLimit'; }
};
