// Auto-generated. Do not edit!

// (in-package estimation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class IRMarker {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.landmarkID = null;
      this.markerID = null;
      this.x = null;
      this.y = null;
      this.z = null;
    }
    else {
      if (initObj.hasOwnProperty('landmarkID')) {
        this.landmarkID = initObj.landmarkID
      }
      else {
        this.landmarkID = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('markerID')) {
        this.markerID = initObj.markerID
      }
      else {
        this.markerID = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IRMarker
    // Serialize message field [landmarkID]
    bufferOffset = std_msgs.msg.String.serialize(obj.landmarkID, buffer, bufferOffset);
    // Serialize message field [markerID]
    bufferOffset = std_msgs.msg.String.serialize(obj.markerID, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IRMarker
    let len;
    let data = new IRMarker(null);
    // Deserialize message field [landmarkID]
    data.landmarkID = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [markerID]
    data.markerID = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.landmarkID);
    length += std_msgs.msg.String.getMessageSize(object.markerID);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'estimation/IRMarker';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '74186740d14e922a7e5ca083a6795f31';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ROS Message that contains pixel-space IR marker locations and labels.
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IRMarker(null);
    if (msg.landmarkID !== undefined) {
      resolved.landmarkID = std_msgs.msg.String.Resolve(msg.landmarkID)
    }
    else {
      resolved.landmarkID = new std_msgs.msg.String()
    }

    if (msg.markerID !== undefined) {
      resolved.markerID = std_msgs.msg.String.Resolve(msg.markerID)
    }
    else {
      resolved.markerID = new std_msgs.msg.String()
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    return resolved;
    }
};

module.exports = IRMarker;
