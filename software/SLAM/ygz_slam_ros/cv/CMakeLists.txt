add_library(ygz-cv
        src/ORBExtractor.cpp
        src/ORBMatcher.cpp
        src/Tracker.cpp
        src/Align.cpp
        src/LKFlow.cpp
        src/TrackerLK.cpp
        )

target_link_libraries(ygz-cv ygz-common ${THIRD_PARTY_LIBS})