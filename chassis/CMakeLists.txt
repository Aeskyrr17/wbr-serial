add_library(chassis_app INTERFACE)
target_include_directories(chassis_app INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
if(BOARD STREQUAL "h723_mc02")
    target_sources(chassis_app INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/app.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/chassis.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/states/relax.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/states/recover.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/states/flatten.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/states/neutral.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/states/normal.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/states/spin.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/states/airborne.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/command/cmd.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/command/control_map.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/model/pendulum.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/leg_mpc.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/solver/tiny_mpc.c
    )
    set_property(SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/solver/tiny_mpc.c
        TARGET_DIRECTORY ${PROJECT_NAME} PROPERTY COMPILE_OPTIONS -O2)
endif()
