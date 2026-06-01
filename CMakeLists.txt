cmake_minimum_required(VERSION 3.10)
project(MultiCamearSync VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
if(MSVC OR CMAKE_GENERATOR STREQUAL "Xcode")
    foreach(OUTPUTCONFIG DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG_UPPER)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
    endforeach()
endif()

if(WIN32)
    set(ARCHITECTURE "win_x64")
elseif(UNIX)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(ARCHITECTURE "linux_x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        set(ARCHITECTURE "linux_arm64")
    endif()
endif()

message(STATUS "Current ARCHITECTURE: ${ARCHITECTURE}")

set(OB_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/orbbecsdk/${ARCHITECTURE}")
set(COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")

# OpenCV
find_package(OpenCV REQUIRED)

# Threads (required for std::thread on Linux; Ubuntu 20.04+ ld --as-needed
# requires explicit pthread linkage, otherwise: "DSO missing from command line")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ============================================================================
# Target 1: MultiDeviceSync (standard multi-device sync)
# ============================================================================
add_executable(MultiDeviceSync
    MultiDeviceSync/MultiDeviceSync.cpp
    common/PipelineHolder.cpp
    common/FramePairingManager.cpp
    common/utils/cJSON.c
    common/utils/utils.cpp
    common/utils/utils_c.c
    common/utils/utils_opencv.cpp
)

target_include_directories(MultiDeviceSync PUBLIC
    "${OB_SDK_DIR}/include/"
    "${COMMON_DIR}"
    "${COMMON_DIR}/utils"
)

target_include_directories(MultiDeviceSync PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(MultiDeviceSync PUBLIC ${OpenCV_LIBS})
target_link_libraries(MultiDeviceSync PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
      
if(WIN32)
    target_link_libraries(MultiDeviceSync PUBLIC "${OB_SDK_DIR}/lib/OrbbecSDK.lib")
    add_custom_command(TARGET MultiDeviceSync POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${OB_SDK_DIR}/bin" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    )
else()
    file(GLOB LIB_ORBBECSDK_FILES "${OB_SDK_DIR}/lib/*.so")
    target_link_libraries(MultiDeviceSync PUBLIC ${LIB_ORBBECSDK_FILES})
    add_custom_command(TARGET MultiDeviceSync POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${OB_SDK_DIR}/lib" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    )
endif()

# ============================================================================
# Target 2: MultiDeviceSyncGmslTrigger (GMSL PWM trigger, Linux only)
# ============================================================================
if(UNIX)
    add_executable(MultiDeviceSyncGmslTrigger
        MultiDeviceSyncGmslTrigger/MultiDeviceSyncGmslTrigger.cpp
    )

    target_include_directories(MultiDeviceSyncGmslTrigger PUBLIC
        "${OB_SDK_DIR}/include/"
    )

    file(GLOB LIB_ORBBECSDK_FILES_GMSL "${OB_SDK_DIR}/lib/*.so")
    target_link_libraries(MultiDeviceSyncGmslTrigger PUBLIC ${LIB_ORBBECSDK_FILES_GMSL})
    target_link_libraries(MultiDeviceSyncGmslTrigger PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
    add_custom_command(TARGET MultiDeviceSyncGmslTrigger POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${OB_SDK_DIR}/lib" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    )
endif()

# ============================================================================
# Copy config files
# ============================================================================
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/OrbbecSDKConfig.xml DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/MultiDeviceSyncConfig.json DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
