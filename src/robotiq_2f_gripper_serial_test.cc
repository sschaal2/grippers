// system includes
#include <cstring>
#include <iostream>
#include <unistd.h>

// local includes
#include "robotiq_2f_gripper.h"
#include "robotiq_2f_gripper_serial.h"

using robotiq_2f_gripper::Robotiq2fGripperSerial;

// minmal test of class methods without connected communication interfacee
bool Robotiq2fGripperSerialTest (int repeats) {
  double position, force;
  char port_name[] = "/dev/ttyUSB1";

  Robotiq2fGripperSerial gripper(0.085, 0., 0.15, 220.,port_name);
  gripper.GripperInitialization();

  
  // blocking command guarantees that the gripper finishes before the next command
  //  gripper.GripperControlCommandBlocking(0.05, 1, 1, 1.0);
  //  gripper.GripperControlCommandBlocking(0.07, 1, 1, 1.0);
  // non-blocking: the next command overwrites and on-going previous command
  //  gripper.GripperControlCommand(0.05, 1, 1);
  //  gripper.GripperControlCommand(0.07, 1, 1);

  for (int i=1; i<=repeats; ++i) {
    gripper.GripperControlCommandBlocking(0.02, 1, 200, 1.0);
    
    std::cout << "Gripper status = " <<
      gripper.GetGripperStatus(&position, &force, 1.0) << std::endl;

    gripper.GripperControlCommandBlocking(0.08, 1, 200, 1.0);
    
    std::cout << "Gripper status = " <<
      gripper.GetGripperStatus(&position, &force, 1.0) << std::endl;

  }

  return true;
}

int
main(int argc, char**argv) {
  int repeats=1;
  if (argc > 1) {
    sscanf(argv[1],"%d",&repeats);
    if (repeats <= 0 || repeats > 100)
      repeats = 1;
  }
  return Robotiq2fGripperSerialTest(repeats);
}


