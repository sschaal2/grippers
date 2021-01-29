//////////////////////////////////////////////////////////////////////////////////  
// Control code for Robotiq 2-finger grippers via Modbus protocol and USB/serial
// communication hardware. This child class just provides the overload for
// serial communication.
//
// Created November 2020
//

#ifndef ROBOTIQ_2F_GRIPPER_SERIAL_H_
#define ROBOTIQ_2F_GRIPPER_SERIAL_H_

// system includes
#include <cstdlib>

// local includes
#include "serial_communication.h"

namespace robotiq_2f_gripper {
  
  ////////////////////////////////////////////////////////////////////////////////
  // the child class for serial communication for two finger grippers
  //
  class Robotiq2fGripperSerial : public Robotiq2fGripper {
  public:
    //////////////////////////////////////////////////////////////////////////////
    // constructor
    //
    // \param [in] max_gripper_width:    gripper fingers can be mechanically
    //                                   configured to different max width
    // \param [in] min_gripper_width:    gripper fingers can be mechanically
    //                                   configured to different min width
    // \param [in] max_gripper_velocity: different Robotiq grippers have different
    //                                   specs for max velocity
    // \param [in] max_gripper_force:    different grippers have different
    //                                   max force
    //
    Robotiq2fGripperSerial(double max_gripper_width,
			   double min_gripper_width,
			   double max_gripper_velocity,
			   double max_gripper_force,
			   char  *serial_port_name);

    //////////////////////////////////////////////////////////////////////////////
    // destructor needs to close communication explicity
    //
    virtual ~Robotiq2fGripperSerial();

  private:

    serial_communication::SerialCommunication *serial_comm_;
    char serial_port_name_[100];

    //////////////////////////////////////////////////////////////////////////////
    // initialize communication
    //
    // returns true/false for successful/failure
    //
    bool InitializeCommunication();

    //////////////////////////////////////////////////////////////////////////////
    // close out communication
    //
    // returns true/false for successful/failure
    //
    bool CloseCommunication();

    //////////////////////////////////////////////////////////////////////////////
    // flush communication
    //
    // returns true/false for successful/failure
    //
    bool FlushCommunication();

    //////////////////////////////////////////////////////////////////////////////
    // send command to gripper
    //
    // \param[in]  modbus_string : complete string to be sent, including CRC
    // \param[in]  len           : length of the entire string
    //
    // returns true/false for successful/failure
    //
    bool SendGripperCommand(uint8_t *modbus_string, size_t len);

    //////////////////////////////////////////////////////////////////////////////
    // receive gripper response
    //
    // \param[in]  modbus_string   : string to be received
    // \param[in]  len             : length of string to be received
    // \param[in]  timeout_seconds : how long, in seconds, to wait for a response
    //
    // returns true/false for successful/failure
    //
    bool ReceiveGripperResponse(uint8_t *modbus_string, size_t len,
					double timeout_seconds);
  };

}  // namespace robotiq_2f_gripper

#endif  // ROBOTIQ_2F_GRIPPER_SERIAL_H_
