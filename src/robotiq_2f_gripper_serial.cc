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

    strcpy(serial_port_name_,serial_port_name);
  }

  //////////////////////////////////////////////////////////////////////////////
  // all necessary jobs are done in base class
  //
  Robotiq2fGripperSerial::~Robotiq2fGripperSerial() {
    CloseCommunication();
  }

  //////////////////////////////////////////////////////////////////////////////
  // the following virtual functions overloads for serial communication
  //
  bool Robotiq2fGripperSerial::InitializeCommunication() {

    // start serial communication
    serial_comm_ = new SerialCommunication(serial_port_name_, 115200, O_RDWR);
    if (!serial_comm_->active_) {
      std::cout << "Serial communication failed to initialize" << std::endl;
      gripper_init_status_ = kGripperUninitialized;      
      return false;
    }
    gripper_init_status_ = kGripperInitialized;
    return true;
  }

  bool Robotiq2fGripperSerial::CloseCommunication() {
    delete serial_comm_;
    return true;
  }

  bool Robotiq2fGripperSerial::FlushCommunication() {
    serial_comm_->clearSerial();
    return true;
  }

  bool Robotiq2fGripperSerial::SendGripperCommand(uint8_t *modbus_string, size_t len){
    size_t len_written = serial_comm_->writeSerial(len,(char *)modbus_string);
    if (len != len_written) {
      printf("Error in SendGripperCommand: not all bytes written: %ld != %ld\n",len,len_written);
      return false;
    }
    return true;
  }

  bool Robotiq2fGripperSerial::ReceiveGripperResponse(uint8_t *modbus_string,
						      size_t len,
						      double timeout_seconds) {
    int count = 0;
    size_t len_read;
    
    while (true) {
      usleep(5000); // sleep at 200Hz
      len_read = (size_t) serial_comm_->checkSerial();
      if (len_read >= len)
	break;
      if (++count > timeout_seconds * 200)
	return false;
    }

    serial_comm_->readSerial(len,(char *)modbus_string);
    
    // timing should be done with usleep() and checked at about 200hz
    // until timeout_seconds are reached
    return true;
  }

}  // namespace robotiq_2f_gripper
