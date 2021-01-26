cc_library(
    name = "robotiq_2f_gripper",
    srcs = [
        "robotiq_2f_gripper.cc",
    ],
    hdrs = glob(["*.h"]),
)

cc_library(
    name = "robotiq_2f_gripper_serial",
    srcs = [
        "robotiq_2f_gripper_serial.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        ":robotiq_2f_gripper",
        "//comm:serial_communication",
    ],    
)

cc_binary(
    name = "robotiq_2f_gripper_test",
    srcs = ["robotiq_2f_gripper_test.cc"],
    deps = [":robotiq_2f_gripper"],
)

cc_binary(
    name = "robotiq_2f_gripper_serial_test",
    srcs = ["robotiq_2f_gripper_serial_test.cc"],
    deps = [":robotiq_2f_gripper_serial"],
)
