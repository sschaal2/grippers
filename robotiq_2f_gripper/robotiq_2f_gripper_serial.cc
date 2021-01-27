// system includes
#include <math.h>
#include <unistd.h>
#include "termios.h"
#include "fcntl.h"


#include <cstring>
#include <iostream>

// local includes
#include "robotiq_2f_gripper.h"
#include "robotiq_2f_gripper_serial.h"
#include "serial_communication.h"

// name spaces
using serial_communication::SerialCommunication;

namespace robotiq_2f_gripper {

  //////////////////////////////////////////////////////////////////////////////
  // intialize base class, and serial communication
  //
  Robotiq2fGripperSerial::Robotiq2fGripperSerial(double max_gripper_width,
						 double min_gripper_width,
						 double max_gripper_velocity,
						 double max_gripper_force,
						 char  *serial_port_name) :
    Robotiq2fGripper(max_gripper_width,
		     min_gripper_width,
		     max_gripper_velocity,
		     max_gripper_force) {
    // start serial communication
    SerialCommunication(serial_port_name, 115200, O_RDWR);
  }

  //////////////////////////////////////////////////////////////////////////////
  // all necessary jobs are done in base class
  //
  Robotiq2fGripperSerial::~Robotiq2fGripperSerial() {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // the following virtual functions are just for debugging
  //  without communiction device
  //
  bool Robotiq2fGripperSerial::InitializeCommunication() {
    std::cout <<
      "Starting communication: no interface has been provided" <<
      "-- communicating with the void" << std::endl;
    gripper_init_status_ =  kGripperNoCommunicationMode;
    return true;
  }

  bool Robotiq2fGripperSerial::CloseCommunication() {
    std::cout << "Closing virtual communication" << std::endl;
    return true;
  }

  bool Robotiq2fGripperSerial::FlushCommunication() {
    std::cout << "Flushing communication" << std::endl;
    return true;
  }

  bool Robotiq2fGripperSerial::SendGripperCommand(uint8_t *modbus_string, size_t len){
    std::cout << "Sending modbus string ";
    for (size_t i=0; i < len; ++i)
      printf("%02x", modbus_string[i]);
    std::cout << std::endl;
    return true;
  }

  bool Robotiq2fGripperSerial::ReceiveGripperResponse(uint8_t *modbus_string,
						      size_t len,
						      double timeout_seconds) {
    std::cerr << "Receiving modbus string" << std::endl;
    // timing should be done with usleep() and checked at about 200hz
    // until timeout_seconds are reached
    return true;
  }

}  // namespace robotiq_2f_gripper
