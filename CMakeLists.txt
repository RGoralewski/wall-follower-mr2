# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(wall_follower)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(WALL_FOLLOWER_LIB_SRC
  src/wall_follower.cpp
)

set(WALL_FOLLOWER_LIB_HEADERS
  include/wall_follower/wall_follower.hpp
  include/wall_follower/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${WALL_FOLLOWER_LIB_SRC}
  ${WALL_FOLLOWER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(WALL_FOLLOWER_NODE_SRC
  src/wall_follower_node.cpp
)

set(WALL_FOLLOWER_NODE_HEADERS
  include/wall_follower/wall_follower_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${WALL_FOLLOWER_NODE_SRC}
  ${WALL_FOLLOWER_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::laboratories::wall_follower::WallFollowerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_wall_follower.cpp)
  set(TEST_WALL_FOLLOWER_EXE test_wall_follower)
  ament_add_gtest(${TEST_WALL_FOLLOWER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_WALL_FOLLOWER_EXE})
  target_link_libraries(${TEST_WALL_FOLLOWER_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
