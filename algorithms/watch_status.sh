source devel/setup.bash
rostopic hz /gaas/localization/registration_pose /gaas/perception/euclidean_original_clusters_list /gaas/navigation/dynamic_block_map /gaas/navigation/target_position #/gaas/localization/IMU_preint_pose #/mavros/setpoint_raw/local
