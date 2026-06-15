load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libcontest_lane_borrow_path.so",
    srcs = [
        "construction_zone_helper.cc",
        "contest_lane_borrow_path.cc",
        "reverse_recovery_helper.cc",
        "u_turn_cone_nudge_helper.cc",
    ],
    hdrs = [
        "construction_zone_helper.h",
        "contest_lane_borrow_path.h",
        "reverse_recovery_helper.h",
        "u_turn_cone_nudge_helper.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/contest_lane_borrow_path/proto:contest_lane_borrow_path_cc_proto",
    ],
)

apollo_package()

cpplint()
