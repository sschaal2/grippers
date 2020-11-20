  //////////////////////////////////////////////////////////////////////////////////  
// Control code for Robotiq 2-finger grippers via Modbus protocol. The
// communication hardware is abstracted, i.e., can be USB/serial, eternet,
// etc., as long as the modbus protocol is used. This base class
// allows position, speed, and force control of the gripper, and can provide
// the gripper status whether an object has been grasped or not, or whether the
// gripper is still in motion or not. The implementaton is lightweight and
// focusses on the most relevant functions of the gripper. Methods for other
// functionalities can be added easily.
//
// Created October 2019
//
// References:
//
// https://robotiq.com/products/2f85-140-adaptive-robot-gripper
// (look at Download files, and get Gripper Instruction Manual)
// A copy is in:
// https://drive.google.com/open?id=1A-X241N93x5kVs4pLU75AVnEw8nd4UkU
//
// A basic desgin rationale is in
// https://drive.google.com/open?id=1imAm7sDdrv2NM-OZSuLAtpxgnTsBXmnqja0Y71uRAtw
//


#ifndef ROBOTIQ_2F_GRIPPER_H_
#define ROBOTIQ_2F_GRIPPER_H_

// system includes
#include <cstdlib>

// google3 header for uint8_t
//#include <bits/stdint-uintn.h>

namespace robotiq_2f_gripper {

// meaningful status variables, derived from bit status responses of the gripper
enum GripperStatus {
  kGripperCommandUnsuccessful = 0,
  kGripperCommandSuccessful,
  kGripperFingersInMotion,
  kGripperFingersStoppedWhileOpening,
  kGripperFingersStoppedWhileClosing,
  kGripperFingersReachedDesiredPosition
};

// meaninful initialization status, including a
// debugging mode without communication
enum GripperInitialization {
  kGripperUninitialized = 0,
  kGripperInitialized,
  kGripperNoCommunicationMode
};

////////////////////////////////////////////////////////////////////////////////
// the base class for two finger grippers
//
class Robotiq2fGripper {
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
  // returns true if initialization was successful, false otherwise
  //
  Robotiq2fGripper(double max_gripper_width,
                   double min_gripper_width,
                   double max_gripper_velocity,
                   double max_gripper_force);

  //////////////////////////////////////////////////////////////////////////////
  // destructor needs to close communication explicity
  //
  virtual ~Robotiq2fGripper();

  //////////////////////////////////////////////////////////////////////////////
  // initialize the tripper, including communication and calibration move
  // returns whether successful or not, and sets gripper_init_sttaus
  //
  // returns true/false for successful/failure
  //
  bool GripperInitialization();

  //////////////////////////////////////////////////////////////////////////////
  // control the gripper with desired position, speed, and force command
  //
  // \param [in] desired_position: the desired finger width
  // \param [in] desired_speed   : the desired absolute speed
  // \param [in] desired_force   : the desired force
  //
  // all values are mapped/clipped into the allowable gripper ranges
  //
  // returns true/false for successful/failure
  //
  GripperStatus GripperControlCommand(double desired_position,
                                      double desired_velocity,
                                      double desired_force);

  //////////////////////////////////////////////////////////////////////////////
  // GripperControlCommand as blocking call, i.e., call returns after
  // gripper movement has finsihed. timeout parameters specifies how
  // long to wait for response
  //
  // returns GripperStatus (see enum)
  //
  GripperStatus GripperControlCommandBlocking(double desired_position,
                                              double desired_velocity,
                                              double desired_force,
                                              double timeout_seconds);

  //////////////////////////////////////////////////////////////////////////////
  // get gripper status and position information
  //
  // \param [out] position       : the desired finger width
  // \param [out] force          : the desired absolute speed
  // \param [in]  timeout_seconds: how long to wait for a response
  //
  // returns GripperStatus (see enum)
  //
  GripperStatus GetGripperStatus(double *position, double *force,
                                 double timeout_seconds);


 private:
  // gripper initizalization status
  enum GripperInitialization gripper_init_status_ = kGripperUninitialized;

  // these variables define the gripper specs, and are used to convert unit
  // variables to uint8_t variables, as needed in modbus command strings
  double max_gripper_force_;
  double max_gripper_width_;
  double min_gripper_width_;
  double max_gripper_velocity_;

  //////////////////////////////////////////////////////////////////////////////
  // compute the CRC of the modbus command string and fix the last two bytes
  // accordingly
  //
  // \param[in]  modbus_string : CRC is computed over len-2 bytes of the string
  // \param[in]  len           : length of the entire string
  //
  void CorrectModbusCRC(uint8_t *modbus_string, size_t len);

  //////////////////////////////////////////////////////////////////////////////
  // initialize communication
  //
  // returns true/false for successful/failure
  //
  virtual bool InitializeCommunication();

  //////////////////////////////////////////////////////////////////////////////
  // close out communication
  //
  // returns true/false for successful/failure
  //
  virtual bool CloseCommunication();

  //////////////////////////////////////////////////////////////////////////////
  // flush communication
  //
  // returns true/false for successful/failure
  //
  virtual bool FlushCommunication();

  //////////////////////////////////////////////////////////////////////////////
  // send command to gripper
  //
  // \param[in]  modbus_string : complete string to be sent, including CRC
  // \param[in]  len           : length of the entire string
  //
  // returns true/false for successful/failure
  //
  virtual bool SendGripperCommand(uint8_t *modbus_string, size_t len);

  //////////////////////////////////////////////////////////////////////////////
  // receive gripper response
  //
  // \param[in]  modbus_string   : string to be received
  // \param[in]  len             : length of string to be received
  // \param[in]  timeout_seconds : how long, in seconds, to wait for a response
  //
  // returns true/false for successful/failure
  //
  virtual bool ReceiveGripperResponse(uint8_t *modbus_string, size_t len,
                                      double timeout_seconds);
};

}  // namespace robotiq_2f_gripper

#endif  // ROBOTIQ_2F_GRIPPER_H_
