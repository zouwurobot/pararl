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

class HomeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.home = null;
    }
    else {
      if (initObj.hasOwnProperty('home')) {
        this.home = initObj.home
      }
      else {
        this.home = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HomeRequest
    // Serialize message field [home]
    bufferOffset = _serializer.int8(obj.home, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HomeRequest
    let len;
    let data = new HomeRequest(null);
    // Deserialize message field [home]
    data.home = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'msgs/HomeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8b7e2f7f747e266548e5f6d35de81285';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 home
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HomeRequest(null);
    if (msg.home !== undefined) {
      resolved.home = msg.home;
    }
    else {
      resolved.home = 0
    }

    return resolved;
    }
};

class HomeResponse {
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
    // Serializes a message object of type HomeResponse
    // Serialize message field [done]
    bufferOffset = _serializer.int8(obj.done, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HomeResponse
    let len;
    let data = new HomeResponse(null);
    // Deserialize message field [done]
    data.done = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'msgs/HomeResponse';
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
    const resolved = new HomeResponse(null);
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
  Request: HomeRequest,
  Response: HomeResponse,
  md5sum() { return '315d885f8f6e90abb232f2881df249a2'; },
  datatype() { return 'msgs/Home'; }
};
