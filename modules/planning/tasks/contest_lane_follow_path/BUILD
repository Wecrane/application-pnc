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
    name = "libcontest_lane_follow_path.so",
    srcs = [
        "contest_lane_follow_path_helper.cc",
        "contest_lane_follow_path.cc",
    ],
    hdrs = [
        "contest_lane_follow_path_helper.h",
        "contest_lane_follow_path.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/contest_lane_follow_path/proto:contest_lane_follow_path_cc_proto",
    ],
)

apollo_package()

cpplint()
