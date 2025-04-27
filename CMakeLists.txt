cmake_minimum_required(VERSION 3.10)
project(hanvon-linux C)
add_executable(hvlusb hanvon-libusb.c)

find_package(PkgConfig)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)
#pkg_check_modules(LIBUDEV REQUIRED libudev)

include_directories(
    ${LIBUSB_INCLUDE_DIRS}
    ${LIBEVDEV_INCLUDE_DIRS}
#    ${LIBUDEV_INCLUDE_DIRS}
)
target_link_libraries(
    hvlusb
    ${LIBUSB_LIBRARIES}
    ${LIBEVDEV_LIBRARIES}
#    ${LIBUDEV_LIBRARIES}
)
