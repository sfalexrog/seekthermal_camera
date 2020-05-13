cmake_minimum_required(VERSION 3.0)
project(libseek_thermal_lite)

# libusb-1.0-dev
find_library(_USB_LIB usb-1.0)
find_path(_USB_INCLUDE NAMES libusb.h PATH_SUFFIXES libusb-1.0)

if(_USB_LIB AND _USB_INCLUDE)
  add_library(Usb INTERFACE)
  target_link_libraries(Usb INTERFACE ${_USB_LIB})
  target_include_directories(Usb INTERFACE ${_USB_INCLUDE})
else()
  message(FATAL_ERROR "Could not find libusb-1.0 development libraries")
endif()

# OpenCV 3+
find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc)

add_library(seek_thermal STATIC
  src/SeekCam.cpp
  src/SeekDevice.cpp
  src/SeekThermal.cpp
  src/SeekThermalPro.cpp
  src/SeekLogging.cpp
)

target_include_directories(seek_thermal
  PRIVATE
    include/seek_thermal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(seek_thermal PUBLIC Usb ${OpenCV_LIBS})

target_compile_options(seek_thermal PRIVATE
  -fpic -fPIC
)
