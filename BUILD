cc_library(
    name = "robotiq_2f_gripper",
    srcs = [
        "src/robotiq_2f_gripper.cc",
    ],
    hdrs = glob(["include/*.h"]),
    includes = ["include"],
)

cc_library(
    name = "robotiq_2f_gripper_serial",
    srcs = [
        "src/robotiq_2f_gripper_serial.cc",
    ],
    hdrs = glob(["include/*.h"]),
    includes = ["include"],    
    deps = [
        ":robotiq_2f_gripper",
        "//comm:serial_communication",
    ],    
)

cc_binary(
    name = "robotiq_2f_gripper_test",
    srcs = ["src/robotiq_2f_gripper_test.cc"],
    includes = ["include"],    
    deps = [":robotiq_2f_gripper"],
)

cc_binary(
    name = "src/robotiq_2f_gripper_serial_test",
    srcs = ["src/robotiq_2f_gripper_serial_test.cc"],
    includes = ["include"],    
    deps = [":robotiq_2f_gripper_serial"],
)
