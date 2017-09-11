cc_library(
    name = "pcl_stuff",
    deps = [
        "@pcl",
        # Targets used.
        "@vtk//:vtkCommonCore",
        "@vtk//:vtkCommonMath",
        "@vtk//:vtkRenderingLOD",
        # Normally private. Why do I have to include these???
        "@vtk//:vtkmetaio",
        "@vtk//:vtklz4",
        "@vtk//:vtkDICOMParser",
        # For not caring:
        "@vtk//:catch_all",
    ],
)


cc_library(
    name = "perception",
    srcs = [
        "perception.cc",
        "point_cloud_registration.cc",
    ] + [
        "perception.h",
        "point_cloud_registration.h",
    ],
    deps = [
        ":pcl_stuff",
        "@drake//drake/lcm",
        "@drake//drake/systems/sensors:image_to_lcm_image_array_t",
        "@drake//drake/perception/estimators/dev:pose_closed_form",
    ],
)

cc_binary(
    name = "link_test",
    srcs = [
        "link_test.cc",
    ],
    deps = [
        ":perception",
    ],
)

cc_binary(
    name = "pose_est_sim",
    srcs = [
        "pose_est_sim.cc",
    ],
    deps = [
        ":perception",
        "@drake//drake/examples/kuka_iiwa_arm/dev/push_pick_place:push_and_pick_demo_lib",
    ],
)

cc_binary(
    name = "plane_seg_dbg",
    srcs = [
        "plane_seg_dbg.cc",
    ],
    data = [
        "pre_subtract.pcd",
    ],
    deps = [
        ":pcl_stuff",
    ],
)
