// Auto-generated. Do not edit!

// (in-package traversability_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class FootprintPath {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.poses = null;
      this.radius = null;
      this.footprint = null;
      this.conservative = null;
      this.compute_untraversable_polygon = null;
    }
    else {
      if (initObj.hasOwnProperty('poses')) {
        this.poses = initObj.poses
      }
      else {
        this.poses = new geometry_msgs.msg.PoseArray();
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('footprint')) {
        this.footprint = initObj.footprint
      }
      else {
        this.footprint = new geometry_msgs.msg.PolygonStamped();
      }
      if (initObj.hasOwnProperty('conservative')) {
        this.conservative = initObj.conservative
      }
      else {
        this.conservative = false;
      }
      if (initObj.hasOwnProperty('compute_untraversable_polygon')) {
        this.compute_untraversable_polygon = initObj.compute_untraversable_polygon
      }
      else {
        this.compute_untraversable_polygon = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FootprintPath
    // Serialize message field [poses]
    bufferOffset = geometry_msgs.msg.PoseArray.serialize(obj.poses, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    // Serialize message field [footprint]
    bufferOffset = geometry_msgs.msg.PolygonStamped.serialize(obj.footprint, buffer, bufferOffset);
    // Serialize message field [conservative]
    bufferOffset = _serializer.bool(obj.conservative, buffer, bufferOffset);
    // Serialize message field [compute_untraversable_polygon]
    bufferOffset = _serializer.bool(obj.compute_untraversable_polygon, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FootprintPath
    let len;
    let data = new FootprintPath(null);
    // Deserialize message field [poses]
    data.poses = geometry_msgs.msg.PoseArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [footprint]
    data.footprint = geometry_msgs.msg.PolygonStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [conservative]
    data.conservative = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [compute_untraversable_polygon]
    data.compute_untraversable_polygon = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseArray.getMessageSize(object.poses);
    length += geometry_msgs.msg.PolygonStamped.getMessageSize(object.footprint);
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traversability_msgs/FootprintPath';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a088c23f98a81d850097dff6ef7145d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new FootprintPath(null);
    if (msg.poses !== undefined) {
      resolved.poses = geometry_msgs.msg.PoseArray.Resolve(msg.poses)
    }
    else {
      resolved.poses = new geometry_msgs.msg.PoseArray()
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.footprint !== undefined) {
      resolved.footprint = geometry_msgs.msg.PolygonStamped.Resolve(msg.footprint)
    }
    else {
      resolved.footprint = new geometry_msgs.msg.PolygonStamped()
    }

    if (msg.conservative !== undefined) {
      resolved.conservative = msg.conservative;
    }
    else {
      resolved.conservative = false
    }

    if (msg.compute_untraversable_polygon !== undefined) {
      resolved.compute_untraversable_polygon = msg.compute_untraversable_polygon;
    }
    else {
      resolved.compute_untraversable_polygon = false
    }

    return resolved;
    }
};

module.exports = FootprintPath;
