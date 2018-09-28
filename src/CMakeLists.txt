# Submodules
include(${CMAKE_SOURCE_DIR}/deps/json.cmake)
include(${CMAKE_SOURCE_DIR}/deps/uWebSockets.cmake)

add_library(core STATIC
    ${CMAKE_SOURCE_DIR}/src/helper_functions.h
    ${CMAKE_SOURCE_DIR}/src/map.h
    ${CMAKE_SOURCE_DIR}/src/particle_filter.h
    ${CMAKE_SOURCE_DIR}/src/particle_filter.cpp
    )

target_include_directories(core
    PUBLIC ${CMAKE_SOURCE_DIR}/src
    )

target_link_libraries(core PUBLIC
    libuWS
    json
    )

if (DO_CLANG_TIDY)
    set_target_properties(core PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
endif ()