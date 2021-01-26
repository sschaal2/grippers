// system includes
#include <cstring>
#include <iostream>

// local includes
#include "robotiq_2f_gripper.h"
#include "robotiq_2f_gripper_serial.h"

using robotiq_2f_gripper::Robotiq2fGripperSerial;

// minmal test of class methods without connected communication interfacee
bool Robotiq2fGripperSerialTest () {
  double position, force;
  char port_name[] = "/port";

  Robotiq2fGripperSerial gripper(1., 0., 1., 1.,port_name);
  gripper.GripperInitialization();
  gripper.GripperControlCommand(0, 1, 1);
  gripper.GripperControlCommandBlocking(0, 1, 1, 1.0);
  std::cout << "Gripper status = " <<
      gripper.GetGripperStatus(&position, &force, 1.0) << std::endl;

  return true;
}

int
main(int argc, char**argv) {
  return Robotiq2fGripperSerialTest();
}


