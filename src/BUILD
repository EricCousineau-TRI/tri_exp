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
        "@drake//drake/examples/kuka_iiwa_arm/dev/push_pick_place:push_and_pick_state_machine",
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
