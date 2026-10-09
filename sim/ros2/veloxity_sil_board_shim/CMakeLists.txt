cmake_minimum_required(VERSION 3.16)
project(veloxity_sil_board_shim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(VELOXITY_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(VELOXITY_SIM_STATICLIB "${VELOXITY_REPO_ROOT}/target/release/libsim.a")

add_custom_target(
  veloxity_sim_staticlib ALL
  COMMAND cargo build -p sim --lib --release
  WORKING_DIRECTORY "${VELOXITY_REPO_ROOT}"
  COMMENT "Building Veloxity sim static library"
  BYPRODUCTS "${VELOXITY_SIM_STATICLIB}"
)

add_library(veloxity_sim STATIC IMPORTED GLOBAL)
set_target_properties(veloxity_sim PROPERTIES
  IMPORTED_LOCATION "${VELOXITY_SIM_STATICLIB}"
)
add_dependencies(veloxity_sim veloxity_sim_staticlib)

add_executable(veloxity_sil_board src/veloxity_sil_board.cpp)
target_compile_features(veloxity_sil_board PRIVATE cxx_std_17)
target_include_directories(veloxity_sil_board PRIVATE include)
ament_target_dependencies(
  veloxity_sil_board
  rclcpp
  rosflight_msgs
  sensor_msgs
  std_srvs
)
target_link_libraries(veloxity_sil_board veloxity_sim pthread dl m)

install(TARGETS veloxity_sil_board
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/verified_param_loader.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
