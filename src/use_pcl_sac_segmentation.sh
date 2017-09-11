#!/bin/bash

pcl_sac_segmentation_plane -thresh 0.001 ./pre_subtract.pcd ./post_subtract.pcd
pcl_viewer ./post_subtract.pcd
