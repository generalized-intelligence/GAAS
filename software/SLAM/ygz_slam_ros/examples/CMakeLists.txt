add_executable(EurocStereo EurocStereo.cpp)
target_link_libraries(EurocStereo
        ${YGZ_LIBS}
        ${THIRD_PARTY_LIBS}
        )

#add_executable(EurocStereoVIO EurocStereoVIO.cpp)
#target_link_libraries(EurocStereoVIO
#        ${YGZ_LIBS}
#        ${THIRD_PARTY_LIBS}
#        )

add_executable(Kitti Kitti.cpp)
target_link_libraries(Kitti
        ${YGZ_LIBS}
        ${THIRD_PARTY_LIBS}
        )
add_executable(EurocStereoVIO_ros EurocStereoVIO_ros.cpp)
target_link_libraries(EurocStereoVIO_ros
        ${YGZ_LIBS}
        ${THIRD_PARTY_LIBS}
        )


add_executable(MakeSceneEurocStereoVIO_ros MakeSceneEurocStereoVIO_ros.cpp)
target_link_libraries(MakeSceneEurocStereoVIO_ros
        ${YGZ_LIBS}
        ${THIRD_PARTY_LIBS}
        )

