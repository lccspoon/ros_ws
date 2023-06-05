// Auto-generated. Do not edit!

// (in-package traversability_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TraversabilityResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_safe = null;
      this.traversability = null;
      this.area = null;
    }
    else {
      if (initObj.hasOwnProperty('is_safe')) {
        this.is_safe = initObj.is_safe
      }
      else {
        this.is_safe = false;
      }
      if (initObj.hasOwnProperty('traversability')) {
        this.traversability = initObj.traversability
      }
      else {
        this.traversability = 0.0;
      }
      if (initObj.hasOwnProperty('area')) {
        this.area = initObj.area
      }
      else {
        this.area = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TraversabilityResult
    // Serialize message field [is_safe]
    bufferOffset = _serializer.bool(obj.is_safe, buffer, bufferOffset);
    // Serialize message field [traversability]
    bufferOffset = _serializer.float64(obj.traversability, buffer, bufferOffset);
    // Serialize message field [area]
    bufferOffset = _serializer.float64(obj.area, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TraversabilityResult
    let len;
    let data = new TraversabilityResult(null);
    // Deserialize message field [is_safe]
    data.is_safe = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [traversability]
    data.traversability = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [area]
    data.area = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traversability_msgs/TraversabilityResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14ffe3323c91cd823bef7a313714954e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # If path is safe to traverse.
    bool is_safe
    
    # Estimate of the traversability of the path.
    # Ranges from 0 to 1 where 0 means not traversable and 1 highly traversable.
    float64 traversability
    
    # Area of the footprint path.
    float64 area
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TraversabilityResult(null);
    if (msg.is_safe !== undefined) {
      resolved.is_safe = msg.is_safe;
    }
    else {
      resolved.is_safe = false
    }

    if (msg.traversability !== undefined) {
      resolved.traversability = msg.traversability;
    }
    else {
      resolved.traversability = 0.0
    }

    if (msg.area !== undefined) {
      resolved.area = msg.area;
    }
    else {
      resolved.area = 0.0
    }

    return resolved;
    }
};

module.exports = TraversabilityResult;
