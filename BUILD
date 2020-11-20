cc_library(
    name = "robotiq_2f_gripper",
    srcs = [
        "robotiq_2f_gripper.cc",
    ],
    hdrs = glob(["*.h"]),
)

cc_binary(
    name = "robotiq_2f_gripper_test",
    srcs = ["robotiq_2f_gripper_test.cc"],
    deps = [":robotiq_2f_gripper"],
)
