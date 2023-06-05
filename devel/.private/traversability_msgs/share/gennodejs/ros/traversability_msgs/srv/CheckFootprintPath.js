// Auto-generated. Do not edit!

// (in-package traversability_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FootprintPath = require('../msg/FootprintPath.js');

//-----------------------------------------------------------

let TraversabilityResult = require('../msg/TraversabilityResult.js');

//-----------------------------------------------------------

class CheckFootprintPathRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path = null;
    }
    else {
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckFootprintPathRequest
    // Serialize message field [path]
    // Serialize the length for message field [path]
    bufferOffset = _serializer.uint32(obj.path.length, buffer, bufferOffset);
    obj.path.forEach((val) => {
      bufferOffset = FootprintPath.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckFootprintPathRequest
    let len;
    let data = new CheckFootprintPathRequest(null);
    // Deserialize message field [path]
    // Deserialize array length for message field [path]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.path = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.path[i] = FootprintPath.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.path.forEach((val) => {
      length += FootprintPath.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'traversability_msgs/CheckFootprintPathRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '715ac8adb4922a1589b635a04ab24fb2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Footprint path to check.
    traversability_msgs/FootprintPath[] path
    
    
    ================================================================================
    MSG: traversability_msgs/FootprintPath
    # Path of the footprint defined as array of connected poses.
    # Poses are connected piece-wise linear in the order of the array.
    geometry_msgs/PoseArray poses
    
    # Either: Define footprint radius.
    float64 radius 
    
    # Or: Define footprint as polygon in the robot base frame.
    # Polygon is used if it is defined, otherwise radius is used.
    geometry_msgs/PolygonStamped footprint
    
    # Use conservative footprint. Only available if polygon is used.
    bool conservative
    
    # Compute untraversable polygon in the checked area for traversability. If true, computation demand is higher.
    bool compute_untraversable_polygon
    
    ================================================================================
    MSG: geometry_msgs/PoseArray
    # An array of poses with a header for global reference.
    
    Header header
    
    Pose[] poses
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/PolygonStamped
    # This represents a Polygon with reference coordinate frame and timestamp
    Header header
    Polygon polygon
    
    ================================================================================
    MSG: geometry_msgs/Polygon
    #A specification of a polygon where the first and last points are assumed to be connected
    Point32[] points
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckFootprintPathRequest(null);
    if (msg.path !== undefined) {
      resolved.path = new Array(msg.path.length);
      for (let i = 0; i < resolved.path.length; ++i) {
        resolved.path[i] = FootprintPath.Resolve(msg.path[i]);
      }
    }
    else {
      resolved.path = []
    }

    return resolved;
    }
};

class CheckFootprintPathResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckFootprintPathResponse
    // Serialize message field [result]
    // Serialize the length for message field [result]
    bufferOffset = _serializer.uint32(obj.result.length, buffer, bufferOffset);
    obj.result.forEach((val) => {
      bufferOffset = TraversabilityResult.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckFootprintPathResponse
    let len;
    let data = new CheckFootprintPathResponse(null);
    // Deserialize message field [result]
    // Deserialize array length for message field [result]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.result = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.result[i] = TraversabilityResult.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 17 * object.result.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'traversability_msgs/CheckFootprintPathResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '403c136f58a3e688827b9a267b872101';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # Traversability results
    traversability_msgs/TraversabilityResult[] result
    
    ================================================================================
    MSG: traversability_msgs/TraversabilityResult
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
    const resolved = new CheckFootprintPathResponse(null);
    if (msg.result !== undefined) {
      resolved.result = new Array(msg.result.length);
      for (let i = 0; i < resolved.result.length; ++i) {
        resolved.result[i] = TraversabilityResult.Resolve(msg.result[i]);
      }
    }
    else {
      resolved.result = []
    }

    return resolved;
    }
};

module.exports = {
  Request: CheckFootprintPathRequest,
  Response: CheckFootprintPathResponse,
  md5sum() { return 'c63bb28cc303394e26806233caf914bd'; },
  datatype() { return 'traversability_msgs/CheckFootprintPath'; }
};
