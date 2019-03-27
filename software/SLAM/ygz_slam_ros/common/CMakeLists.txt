add_library(ygz-common
        src/IMUData.cpp
        src/Settings.cpp
        src/Frame.cpp
        src/MapPoint.cpp
        src/G2OTypes.cpp
        src/utility.cpp
	src/scene_retrieve.cpp
        )

target_link_libraries(ygz-common
        ${THIRD_PARTY_LIBS}
        )
