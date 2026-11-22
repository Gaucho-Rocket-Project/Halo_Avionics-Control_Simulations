cmake_minimum_required(VERSION 3.23)

project(FlightControl VERSION 1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(flight_control
    "src/lqr/lqr.c"
    "src/lqr/lqr_config.c"
    "src/vehicle_controller.c"
    "src/roll_control/roll_control.c"
    "src/roll_control/roll_config.c"
    "src/kalman_filter/kalman.c"
    "src/kalman_filter/kalman_config.c"
    "src/trajectory/trajectory.c"
    "src/trajectory/trajectory_config.c"
)

target_include_directories(flight_control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(lqr_test
    "tests/lqr_test.c"
)

target_link_libraries(lqr_test PRIVATE
    flight_control
)

add_executable(vehicle_test
    "tests/vehicle_test.c"
)

target_link_libraries(vehicle_test PRIVATE
    flight_control
)

add_executable(roll_test 
  "tests/roll_test.c"
)

target_link_libraries(roll_test PRIVATE 
  flight_control
)


enable_testing()
add_test(NAME lqr_checks COMMAND lqr_test)
add_test(NAME vehicle_checks COMMAND vehicle_test)
add_test(NAME roll_checks COMMAND roll_test)
