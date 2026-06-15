load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob(["conf/**"]),
)

apollo_plugin(
    name = "libcontest_scenario.so",
    srcs = [
        "contest_scenario_util.cc",
        "contest_scenarios.cc",
        "stage_contest_lane_follow.cc",
    ],
    hdrs = [
        "contest_scenario_util.h",
        "contest_scenarios.h",
        "context.h",
        "stage_contest_lane_follow.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/math:math",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/contest/proto:contest_cc_proto",
        "//modules/planning/scenarios/lane_follow:lane_follow_scenario_lib",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package(enable_source=True)

cpplint()
