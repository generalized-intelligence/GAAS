add_library(ygz-system
        src/System.cpp
)

target_link_libraries( ygz-system
    ygz-common ygz-cv ygz-util ygz-backend
)