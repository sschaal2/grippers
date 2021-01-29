// system includes
#include <math.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

// local includes
#include "robotiq_2f_gripper.h"

namespace robotiq_2f_gripper {

//////////////////////////////////////////////////////////////////////////////
// sets gripper parameters
//
Robotiq2fGripper::Robotiq2fGripper(double max_gripper_width,
                                   double min_gripper_width,
                                   double max_gripper_velocity,
                                   double max_gripper_force) {
  max_gripper_force_    = max_gripper_force;
  max_gripper_width_    = max_gripper_width;
  min_gripper_width_    = min_gripper_width;
  max_gripper_velocity_ = max_gripper_velocity;
}

//////////////////////////////////////////////////////////////////////////////
// need to close out the communication
//
Robotiq2fGripper::~Robotiq2fGripper() {
  CloseCommunication();
}

//////////////////////////////////////////////////////////////////////////////
// the entire initialization sequence according to robotiq manual
//
bool Robotiq2fGripper::GripperInitialization() {
  // all modbus commands strings that are relevant
  uint8_t modbus_clear_rACT[] =
      {0x09,0x10,0x03,0xE8,0x00,0x03,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint8_t modbus_clear_rACT_expected_response[] =
      {0x09,0x10,0x03,0xE8,0x00,0x03,0x01,0x30};
  uint8_t modbus_clear_rACT_response[sizeof(modbus_clear_rACT_expected_response)];

  uint8_t modbus_set_rACT[] =
      {0x09,0x10,0x03,0xE8,0x00,0x03,0x06,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint8_t modbus_set_rACT_expected_response[] =
      {0x09,0x10,0x03,0xE8,0x00,0x03,0x01,0x30};
  uint8_t modbus_set_rACT_response[sizeof(modbus_set_rACT_expected_response)];

  uint8_t modbus_read_status[] = {0x09,0x03,0x07,0xD0,0x00,0x01,0x85,0xCF};
  uint8_t modbus_read_status_response_not_activated[] =
      {0x09,0x03,0x02,0x11,0x00,0x55,0xD5};
  uint8_t modbus_read_status_response_is_activated[] =
      {0x09,0x03,0x02,0x31,0x00,0x4C,0x15};
  uint8_t modbus_read_status_response[sizeof(modbus_read_status_response_is_activated)];

  // whatever communication protocol,
  // it should be initialized after this command
  if (!InitializeCommunication())
    return false;

  // generate the sequence of initialization commands -- see robotiq manual to
  // get the modbus commands explained.
  double time_out = 10;

  // clear all faults, i.e., clear rACT
  CorrectModbusCRC(modbus_clear_rACT, sizeof(modbus_clear_rACT));
  FlushCommunication();

  if (!SendGripperCommand(modbus_clear_rACT, sizeof(modbus_clear_rACT)))
    return false;

  // just for debugging without communication
  if (gripper_init_status_ ==  kGripperNoCommunicationMode)
    memcpy(modbus_clear_rACT_response, modbus_clear_rACT_expected_response,
           sizeof(modbus_clear_rACT_response));

  if (!ReceiveGripperResponse(modbus_clear_rACT_response,
                              sizeof(modbus_clear_rACT_response), time_out))
    return false;

  if (memcmp(modbus_clear_rACT_response, modbus_clear_rACT_expected_response,
             sizeof(modbus_clear_rACT_response)) != 0) {
    printf("modbus_clear_rACT_response is wrong\n");
    return false;
  }


  // set rACT: this triggers a calibration motion
  CorrectModbusCRC(modbus_set_rACT, sizeof(modbus_set_rACT));
  FlushCommunication();
  if (!SendGripperCommand(modbus_set_rACT, sizeof(modbus_set_rACT)))
    return false;

  // just for debugging without communication
  if (gripper_init_status_ ==  kGripperNoCommunicationMode)
    memcpy(modbus_set_rACT_response, modbus_set_rACT_expected_response,
           sizeof(modbus_set_rACT_response));

  if (!ReceiveGripperResponse(modbus_set_rACT_response,
                              sizeof(modbus_set_rACT_response), time_out)) 
    return false;
  
  if (memcmp(modbus_set_rACT_response, modbus_set_rACT_expected_response,
             sizeof(modbus_set_rACT_response)) != 0) {
    printf("modbus_sete_rACT_response is wrong\n");    
    return false;
  }

  // read gripper status and wait for initialization
  CorrectModbusCRC(modbus_read_status, sizeof(modbus_read_status));
  FlushCommunication();
  if (!SendGripperCommand(modbus_read_status, sizeof(modbus_read_status)))
    return false;

  // just for debugging without communication
  if (gripper_init_status_ ==  kGripperNoCommunicationMode)
    memcpy(modbus_read_status_response,
           modbus_read_status_response_is_activated,
           sizeof(modbus_read_status_response));

  // wait a few seconds before giving up: only is_actived and
  // not_activated respones are allowed
  int counter = 10;
  do {
    if (!ReceiveGripperResponse(modbus_read_status_response,
                                sizeof(modbus_read_status_response), time_out))
      return false;

    if (memcmp(modbus_read_status_response,
               modbus_read_status_response_is_activated,
               sizeof(modbus_read_status_response)) == 0)
      break;

    if (memcmp(modbus_read_status_response,
               modbus_read_status_response_not_activated,
               sizeof(modbus_read_status_response)) != 0) {
      printf("modbus_read_status_response is wrong\n");          
      return false;
    }
  } while (--counter > 0);


  // finally initialized properly
  if (gripper_init_status_ != kGripperNoCommunicationMode)
    gripper_init_status_ = kGripperInitialized;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// the following virtual functions are just for debugging
//  without communiction device
//
bool Robotiq2fGripper::InitializeCommunication() {
  std::cout <<
      "Starting communication: no interface has been provided" <<
      "-- communicating with the void" << std::endl;
  gripper_init_status_ =  kGripperNoCommunicationMode;
  return true;
}

bool Robotiq2fGripper::CloseCommunication() {
  std::cout << "Closing virtual communication" << std::endl;
  return true;
}

bool Robotiq2fGripper::FlushCommunication() {
  std::cout << "Flushing communication" << std::endl;
  return true;
}

bool Robotiq2fGripper::SendGripperCommand(uint8_t *modbus_string, size_t len){
  std::cout << "Sending modbus string ";
  for (size_t i=0; i < len; ++i)
    printf("%02x", modbus_string[i]);
  std::cout << std::endl;
  return true;
}

bool Robotiq2fGripper::ReceiveGripperResponse(uint8_t *modbus_string,
                                              size_t len,
                                              double timeout_seconds) {
  std::cerr << "Receiving modbus string" << std::endl;
  // timing should be done with usleep() and checked at about 200hz
  // until timeout_seconds are reached
  return true;
}

//////////////////////////////////////////////////////////////////////////////
// copied from public information -- search for "crc modbus rtu"
//
void Robotiq2fGripper::CorrectModbusCRC(uint8_t *modbus_string, size_t len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len-2; pos++) {
    crc ^= modbus_string[pos];        // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                        // Else LSB is not set
        crc >>= 1;                    // Just shift right
      }
    }
  }

  // now fix the last two bytes with checksum
  modbus_string[len-1] = crc >> 8;
  modbus_string[len-2] = (crc << 8) >> 8;
}


//////////////////////////////////////////////////////////////////////////////
// the key API: moving the gripper to a position,
// with speed and force specifications
//
GripperStatus Robotiq2fGripper::GripperControlCommand(double desired_position,
                                                      double desired_velocity,
                                                      double desired_force) {
  // all modbus commands strings that are relevant
  uint8_t modbus_move[] =
      {0x09,0x10,0x03,0xE8,0x00,0x03,0x06,0x09,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00};
  uint8_t modbus_move_expected_response[] =
      {0x09,0x10,0x03,0xE8,0x00,0x03,0x01,0x30};
  uint8_t modbus_move_response[sizeof(modbus_move_expected_response)];

  // correct the bytes for position, velocity, and force
  // in the modbus move command

  // position byte: (0xff is fully closed, position is gripper opening width)
  double normalized_finger_distance;
  normalized_finger_distance = (desired_position - min_gripper_width_)/
      (max_gripper_width_ - min_gripper_width_);
  if (normalized_finger_distance > 1.0)
    normalized_finger_distance = 1.0;
  else if (normalized_finger_distance < 0)
    normalized_finger_distance = 0;

  modbus_move[10] = (uint8_t)((1.0-normalized_finger_distance)*0xff);

  // velocity byte: (0xff is full velocity)
  double normalized_velocity;
  normalized_velocity = fabs(desired_velocity/max_gripper_velocity_);
  if (normalized_velocity > 1.0)
    normalized_velocity = 1.0;
  modbus_move[11] = (uint8_t)(normalized_velocity*0xff);

  // force byte  (0xff is full force)
  double normalized_force;
  normalized_force = fabs(desired_force/max_gripper_force_);
  if (normalized_force > 1.0)
    normalized_force = 1.0;
  modbus_move[12] = (uint8_t)(normalized_force*0xff);

  // trigger the movee
  CorrectModbusCRC(modbus_move, sizeof(modbus_move));
  FlushCommunication();
  if (!SendGripperCommand(modbus_move, sizeof(modbus_move)))
    return kGripperCommandUnsuccessful;

  // just for debugging without communication
  if (gripper_init_status_ ==  kGripperNoCommunicationMode)
    memcpy(modbus_move_response, modbus_move_expected_response,
           sizeof(modbus_move_response));

  // as this is non-blocking, the result of the read is not
  // check for the correct response
  ReceiveGripperResponse(modbus_move_response,
                         sizeof(modbus_move_response), 0.0);

  return kGripperCommandSuccessful;
}


//////////////////////////////////////////////////////////////////////////////
// the key API: moving the gripper to a position,
// with speed and force specifications
//
GripperStatus Robotiq2fGripper::
    GripperControlCommandBlocking(double desired_position,
                                  double desired_velocity,
                                  double desired_force,
                                  double timeout_seconds) {
  // 200Hz  is what  Robotiq recommends as highest frequency
  double        sleep_time = 0.005;
  int           counter    = timeout_seconds/sleep_time;
  GripperStatus status     = kGripperFingersInMotion;
  double        position, force;

  // start the gripper motion
  if (GripperControlCommand(desired_position, desired_velocity, desired_force)
      == kGripperCommandUnsuccessful)
    return kGripperCommandUnsuccessful;

  // read status as long as fingers are moving
  do {
    status = GetGripperStatus(&position, &force, sleep_time);
  } while (--counter > 0 && status == kGripperFingersInMotion);

  return status;
}


//////////////////////////////////////////////////////////////////////////////
// get the gripper status
//
GripperStatus Robotiq2fGripper::GetGripperStatus(double *position,
                                                 double *force,
                                                 double timeout_seconds) {
  // all modbus commands strings that are relevant
  uint8_t modbus_read_status[] = {0x09,0x03,0x07,0xD0,0x00,0x03,0x04,0x0E};
  uint8_t modbus_read_status_response[11];

  // request status
  CorrectModbusCRC(modbus_read_status, sizeof(modbus_read_status));
  FlushCommunication();
  if (!SendGripperCommand(modbus_read_status, sizeof(modbus_read_status)))
    return kGripperCommandUnsuccessful;

  // read response until timeout is reached
  double sleep_time = 0.005;
  int    counter    = timeout_seconds/sleep_time;
  do {
    if (ReceiveGripperResponse(modbus_read_status_response,
                               sizeof(modbus_read_status_response),
                               sleep_time))
      break;
  } while (--counter > 0);

  if (counter <= 0)
    return kGripperCommandUnsuccessful;

  // analyze the return bytes to extract status
  uint8_t status_byte = modbus_read_status_response[3];
  uint8_t gOBJ = (0b11000000 && status_byte) >> 6;

  *position = ((double)(modbus_read_status_response[7])/
               (double)(0xff))*(max_gripper_width_ - min_gripper_width_) +
      min_gripper_width_;
  *force = ((double)(modbus_read_status_response[7])/
            (double)(0xff))*max_gripper_force_;

  if (gOBJ == 0)
    return kGripperFingersInMotion;
  else if (gOBJ == 1)
    return kGripperFingersStoppedWhileOpening;
  else if (gOBJ == 2)
    return kGripperFingersStoppedWhileClosing;
  else if (gOBJ == 3)
    return kGripperFingersReachedDesiredPosition;
  else
    return kGripperCommandUnsuccessful;  // should not be possible
}


}  // namespace robotiq_2f_gripper
