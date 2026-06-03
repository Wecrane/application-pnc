load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libbus_bay_transfer_scenario.so",
    srcs = [
        "stage_approaching_bus_bay.cc",
        "stage_departing_from_bus_bay.cc",
        "stage_dwelling_at_bus_bay.cc",
        "bus_bay_transfer_scenario.cc",
    ],
    hdrs = [
        "stage_approaching_bus_bay.h",
        "stage_departing_from_bus_bay.h",
        "stage_dwelling_at_bus_bay.h",
        "bus_bay_transfer_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/valet_parking/proto:bus_bay_transfer_cc_proto",
    ],
)

apollo_package()

cpplint()
