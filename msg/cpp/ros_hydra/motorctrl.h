/* auto-generated by genmsg_cpp from /home/floris/ros/stacks/ros_hydra/msg/motorctrl.msg.  Do not edit! */
#ifndef ROS_HYDRA_MOTORCTRL_H
#define ROS_HYDRA_MOTORCTRL_H

#include <string>
#include <vector>
#include "ros/message.h"
#include "ros/debug.h"
#include "ros/assert.h"
#include "ros/time.h"

namespace ros_hydra
{

//! \htmlinclude motorctrl.msg.html

class motorctrl : public ros::Message
{
public:
  typedef boost::shared_ptr<motorctrl> Ptr;
  typedef boost::shared_ptr<motorctrl const> ConstPtr;

  typedef double _pos_type;
  typedef double _vel_type;

  double pos;
  double vel;

  motorctrl() : ros::Message(),
    pos(0),
    vel(0)
  {
  }
  motorctrl(const motorctrl &copy) : ros::Message(),
    pos(copy.pos),
    vel(copy.vel)
  {
    (void)copy;
  }
  motorctrl &operator =(const motorctrl &copy)
  {
    if (this == &copy)
      return *this;
    pos = copy.pos;
    vel = copy.vel;
    return *this;
  }
  virtual ~motorctrl() 
  {
  }
  inline static std::string __s_getDataType() { return std::string("ros_hydra/motorctrl"); }
  inline static std::string __s_getMD5Sum() { return std::string("ee525ac03d9b74c06a89b18b5a69ac81"); }
  inline static std::string __s_getMessageDefinition()
  {
    return std::string(
    "float64 pos\n"
    "float64 vel\n"
    "\n"
    "\n"
    );
  }
  inline virtual const std::string __getDataType() const { return __s_getDataType(); }
  inline virtual const std::string __getMD5Sum() const { return __s_getMD5Sum(); }
  inline virtual const std::string __getMessageDefinition() const { return __s_getMessageDefinition(); }
  inline uint32_t serializationLength() const
  {
    unsigned __l = 0;
    __l += 8; // pos
    __l += 8; // vel
    return __l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr,
#if defined(__GNUC__)
                             __attribute__((unused)) uint32_t seq) const
#else
                             uint32_t seq) const
#endif
  {
    SROS_SERIALIZE_PRIMITIVE(write_ptr, pos);
    SROS_SERIALIZE_PRIMITIVE(write_ptr, vel);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, pos);
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, vel);
    return read_ptr;
  }
};

typedef boost::shared_ptr<motorctrl> motorctrlPtr;
typedef boost::shared_ptr<motorctrl const> motorctrlConstPtr;


}

#endif
