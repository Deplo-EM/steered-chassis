# ##############################################################################
# #################        CMake Template (CUSTOM)       ######################
# #################    Copyright (c) 2023 Hello World    ######################
# ##############################################################################

# Set the system name and version
set(CMAKE_SYSTEM_VERSION 1)

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.22)

include("cmake/gcc-arm-none-eabi.cmake")

# Set the C++ and C standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 使用 HAL 的毫秒滴答（HAL_GetTick）作为 tick.cpp 的时间基准。
# tick.cpp 内部按宏选择实现，不定义这个宏的话 GetTickMs() 根本不会被编译出来，
# 链接时会报 "undefined reference to GetTickMs"。
add_compile_definitions(USE_HAL_TICK)

if(NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build")
endif()
option(ROBOT_STEER_MODE_PROBE "Build a stand-only GM6020 diagnostic, separately from acceptance firmware" OFF)
option(ROBOT_STEER_CURRENT_FIELD "Build a separate chassis firmware for GM6020 axes with the current loop already enabled" OFF)
option(ROBOT_BLOCKED_WHEEL_FALLBACK "Field accommodation: after alignment timeout, release aligned wheels individually (one steer is mechanically blocked)" OFF)
set(ROBOT_EXCLUDED_WHEEL -1 CACHE STRING "Exclude one wheel index (0-3) from angle-fault/alignment/stall/output; -1 disables")
option(ROBOT_STEER_NO_FLIP "Disable the equivalent-solution steer flip (always direct; long arc on reversal)" OFF)
option(ROBOT_STEER_FRICTION_COMP "Add static-friction feedforward on steer axes (bench tuning)" OFF)
option(ROBOT_STEER_BENCH_NO_STALL "Bench-only: disable steer stall latch so friction can be tuned continuously" OFF)
option(ROBOT_ACCEPT_STUCK_STEER "Ground: accept a mechanically stuck steer (no alignment gate, no stall latch); four wheels drive" OFF)
option(ROBOT_SKIP_ANGLE_FAULT "Diagnostic only: skip the steer angle out-of-range fault" OFF)
option(ROBOT_STEER_BREAKAWAY "Breakaway pulse on a stuck steer (no continuous friction feedforward)" OFF)
option(ROBOT_GROUND_TRIAL "Build the separate four-wheel low-speed ground trial profile" OFF)
option(ROBOT_GROUND_TRIAL_MOTION "Build a motion-capable ground trial after physical release checks" OFF)
option(ROBOT_COMPETITION_FINISH "Build the separate three-mode acceptance candidate" OFF)
if(ROBOT_GROUND_TRIAL_MOTION AND NOT ROBOT_GROUND_TRIAL)
  message(FATAL_ERROR "Ground trial motion requires ROBOT_GROUND_TRIAL")
endif()
if(ROBOT_GROUND_TRIAL AND ROBOT_COMPETITION_FINISH)
  message(FATAL_ERROR "Ground trial and competition finish are distinct profiles")
endif()
if((ROBOT_GROUND_TRIAL OR ROBOT_COMPETITION_FINISH) AND
    (NOT ROBOT_STEER_CURRENT_FIELD OR ROBOT_STEER_MODE_PROBE OR
     NOT "${ROBOT_EXCLUDED_WHEEL}" STREQUAL "-1" OR ROBOT_BLOCKED_WHEEL_FALLBACK OR
     ROBOT_STEER_NO_FLIP OR
     ROBOT_SKIP_ANGLE_FAULT OR ROBOT_STEER_BREAKAWAY))
  message(FATAL_ERROR "Ground trial / competition finish requires current steering, all four wheels, alignment, flip and stall protection")
endif()
if(ROBOT_STEER_MODE_PROBE AND ROBOT_STEER_CURRENT_FIELD)
  message(FATAL_ERROR "Select either the steering mode probe or the field current profile")
endif()
if(ROBOT_STEER_MODE_PROBE OR ROBOT_STEER_CURRENT_FIELD)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()
get_filename_component(CMAKE_CURRENT_SOURCE_PARENT_DIR
                       ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

# ########################## USER CONFIG SECTION ##############################
# Set the project name and the languages used
project(steered_chassis C CXX ASM) # TODO: change project name here

# Specify user folders TODO Add your own user folders here
# 新人的代码目录写在这里（需先手动创建对应文件夹），例如：
#   set(user_folders "Instance" "Task" "RobotModules")
set(user_folders "Instance" "RobotModules" "Task" "RobotComponents")
set(user_folders_in_parent_folder "RobotComponents")

# 收集 root_dir 及其所有子目录，作为头文件搜索路径
function(collect_includes root_dir res_list)
  set(dirs ${root_dir})
  file(GLOB_RECURSE children LIST_DIRECTORIES true RELATIVE ${root_dir}
       ${root_dir}/*)
  foreach(child ${children})
    if(IS_DIRECTORY ${root_dir}/${child})
      list(APPEND dirs ${root_dir}/${child})
    endif()
  endforeach()
  set(${res_list}
      ${dirs}
      PARENT_SCOPE)
endfunction()

# Enable preprocessing for assembler files
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# Disable some warnings
set(COM_FLAGS
    "-Wno-unused-parameter -Wno-missing-field-initializers -Wno-pedantic -Wno-unknown-pragmas"
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COM_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COM_FLAGS} -Wno-reorder")

add_subdirectory(cmake/stm32cubemx)
get_target_property(STM32_COMPILE_DEFINES stm32cubemx
                    INTERFACE_COMPILE_DEFINITIONS)
list(APPEND STM32_COMPILE_DEFINES DEBUG) # Add DEBUG definition
foreach(STM32_COMPILE_DEFINE ${STM32_COMPILE_DEFINES})
  # Exclude the element if it starts with "$"
  if(STM32_COMPILE_DEFINE MATCHES "^\\$")
    continue()
  endif()
  add_definitions(-D${STM32_COMPILE_DEFINE})

  # Get HAL filename
  if(STM32_COMPILE_DEFINE MATCHES "STM32[A-Z][0-9]")
    string(SUBSTRING ${STM32_COMPILE_DEFINE} 0 7 STM32_DEVICE)
    string(TOLOWER ${STM32_DEVICE} stm32_hal_filename)
    string(CONCAT stm32_hal_filename ${stm32_hal_filename} "xx_hal.h")
    message(STATUS "HAL file name: ${stm32_hal_filename}")
    # Set macro for the HAL filename
    add_definitions(-DSTM32_HAL_FILENAME="${stm32_hal_filename}")
  endif()
endforeach()

# #################### ADD LIBRARIES AND EXECUTABLE SECTION ####################
# Initialize source and include lists
set(project_srcs)
set(project_incs)

# Core 目录
collect_includes("${CMAKE_CURRENT_SOURCE_DIR}/Core" core_incs)
file(GLOB_RECURSE core_srcs "Core/*.c")
list(APPEND project_srcs ${core_srcs})
list(APPEND project_incs ${core_incs})

# Drivers 目录（STM32 HAL / CMSIS）
collect_includes("${CMAKE_CURRENT_SOURCE_DIR}/Drivers" drivers_incs)
file(GLOB_RECURSE drivers_srcs "Drivers/*.c")
list(APPEND project_incs ${drivers_incs})

# 组件目录：inc/ 放头文件，src/ 放源文件
collect_includes("${CMAKE_CURRENT_SOURCE_DIR}/inc" component_incs)
file(GLOB_RECURSE component_srcs "src/*.c*")
list(APPEND project_srcs ${component_srcs})
list(APPEND project_incs ${component_incs})

# 新人自定义目录
foreach(user_folder ${user_folders})
  list(FIND user_folders_in_parent_folder ${user_folder} INDEX)

  if(${INDEX} GREATER -1)
    message(STATUS "${user_folder} is in the parent folder list")
    set(search_path "${CMAKE_CURRENT_SOURCE_PARENT_DIR}/${user_folder}")
  else()
    set(search_path "${CMAKE_CURRENT_SOURCE_DIR}/${user_folder}")
  endif()

  if(NOT EXISTS ${search_path})
    message(FATAL_ERROR "${user_folder}(${search_path}) does not exist")
  endif()

  collect_includes("${search_path}" ${user_folder}_incs)
  file(GLOB_RECURSE ${user_folder}_srcs "${search_path}/*.c*")
  list(APPEND project_incs ${${user_folder}_incs})
  list(APPEND project_srcs ${${user_folder}_srcs})
endforeach()

# Add a static library for the drivers
add_library(drivers STATIC ${drivers_srcs})
target_include_directories(drivers PUBLIC ${core_incs} ${drivers_incs})

# Add the third party libraries (CMSIS-DSP / Eigen3 / SEGGER RTT)
add_subdirectory(third_party)

# Add an executable for the project
file(GLOB startup_file "${CMAKE_CURRENT_SOURCE_DIR}/*.s")
add_executable(${PROJECT_NAME} ${project_srcs} ${startup_file})
set(firmware_name ${PROJECT_NAME})
if(ROBOT_STEER_MODE_PROBE)
  set(firmware_name steered_chassis_steer_probe)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_STEER_MODE_PROBE=1)
elseif(ROBOT_STEER_CURRENT_FIELD)
  set(firmware_name steered_chassis_steer_current_field)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_STEER_CURRENT_FIELD=1)
endif()
if(ROBOT_BLOCKED_WHEEL_FALLBACK)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_BLOCKED_WHEEL_FALLBACK=1)
endif()
if(ROBOT_EXCLUDED_WHEEL GREATER -1)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_EXCLUDED_WHEEL=${ROBOT_EXCLUDED_WHEEL})
endif()
if(ROBOT_STEER_NO_FLIP)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_STEER_NO_FLIP=1)
endif()
if(ROBOT_STEER_FRICTION_COMP)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_STEER_FRICTION_COMP=1)
endif()
if(ROBOT_STEER_BENCH_NO_STALL)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_STEER_BENCH_NO_STALL=1)
endif()
if(ROBOT_ACCEPT_STUCK_STEER)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_ACCEPT_STUCK_STEER=1)
endif()
if(ROBOT_SKIP_ANGLE_FAULT)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_SKIP_ANGLE_FAULT=1)
endif()
if(ROBOT_STEER_BREAKAWAY)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_STEER_BREAKAWAY=1)
endif()
if(ROBOT_GROUND_TRIAL)
  set(firmware_name steered_chassis_ground_locked)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_GROUND_TRIAL=1)
  if(ROBOT_GROUND_TRIAL_MOTION)
    set(firmware_name steered_chassis_ground_motion)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_GROUND_TRIAL_MOTION=1)
  endif()
endif()
if(ROBOT_COMPETITION_FINISH)
  set(firmware_name steered_chassis_competition_finish)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_COMPETITION_FINISH=1)
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${firmware_name})
message(STATUS "Project name: ${PROJECT_NAME}")

# Add the project includes to the executable
target_include_directories(${PROJECT_NAME} PUBLIC ${project_incs})

# Link the drivers library and the third party libraries to the executable
target_link_libraries(${PROJECT_NAME} PUBLIC drivers)
target_link_libraries(${PROJECT_NAME} PUBLIC cmsis_dsp)

# Define the output hex and bin files
set(HEX_FILE ${PROJECT_BINARY_DIR}/${firmware_name}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${firmware_name}.bin)

# Add a post-build command to generate the hex and bin files
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${BIN_FILE}
  COMMENT "Building ${HEX_FILE} Building ${BIN_FILE}")
