add_library(chassis_app INTERFACE)
target_include_directories(chassis_app INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
if(BOARD STREQUAL "h723_mc02")
    target_sources(chassis_app INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/chassis.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/command/cmd.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/command/control_map.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/model/pendulum.cpp
    )
endif()
