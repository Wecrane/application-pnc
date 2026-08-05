load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "scene_speed_limit_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libscene_speed_limit.so",
    srcs = [
        "scene_speed_limit.cc",
    ],
    hdrs = [
        "scene_speed_limit.h",
    ],
    description = ":plugin_scene_speed_limit_description.xml",
    deps = [
        "//cyber",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/traffic_rules/scene_speed_limit/proto:scene_speed_limit_proto",
    ],
)

apollo_package()

cpplint()