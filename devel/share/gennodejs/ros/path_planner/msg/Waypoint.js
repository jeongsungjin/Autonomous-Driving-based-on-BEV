// Auto-generated. Do not edit!

// (in-package path_planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Waypoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cnt = null;
      this.x_arr = null;
      this.y_arr = null;
    }
    else {
      if (initObj.hasOwnProperty('cnt')) {
        this.cnt = initObj.cnt
      }
      else {
        this.cnt = 0;
      }
      if (initObj.hasOwnProperty('x_arr')) {
        this.x_arr = initObj.x_arr
      }
      else {
        this.x_arr = [];
      }
      if (initObj.hasOwnProperty('y_arr')) {
        this.y_arr = initObj.y_arr
      }
      else {
        this.y_arr = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Waypoint
    // Serialize message field [cnt]
    bufferOffset = _serializer.int32(obj.cnt, buffer, bufferOffset);
    // Serialize message field [x_arr]
    bufferOffset = _arraySerializer.float64(obj.x_arr, buffer, bufferOffset, null);
    // Serialize message field [y_arr]
    bufferOffset = _arraySerializer.float64(obj.y_arr, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Waypoint
    let len;
    let data = new Waypoint(null);
    // Deserialize message field [cnt]
    data.cnt = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [x_arr]
    data.x_arr = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [y_arr]
    data.y_arr = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.x_arr.length;
    length += 8 * object.y_arr.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'path_planner/Waypoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b2ba581b0127fa88549ceec9fea43516';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 cnt
    float64[] x_arr
    float64[] y_arr
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Waypoint(null);
    if (msg.cnt !== undefined) {
      resolved.cnt = msg.cnt;
    }
    else {
      resolved.cnt = 0
    }

    if (msg.x_arr !== undefined) {
      resolved.x_arr = msg.x_arr;
    }
    else {
      resolved.x_arr = []
    }

    if (msg.y_arr !== undefined) {
      resolved.y_arr = msg.y_arr;
    }
    else {
      resolved.y_arr = []
    }

    return resolved;
    }
};

module.exports = Waypoint;
