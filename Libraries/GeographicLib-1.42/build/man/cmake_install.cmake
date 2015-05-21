# Install script for directory: /Users/Maverick/Documents/GeographicLib-1.42/man

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/man/man1" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/CartConvert.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/ConicProj.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/GeodesicProj.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/GeoConvert.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/GeodSolve.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/GeoidEval.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/Gravity.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/MagneticField.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/Planimeter.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/RhumbSolve.1"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/TransverseMercatorProj.1"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/man/man8" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/geographiclib-get-geoids.8"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/geographiclib-get-gravity.8"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/man/geographiclib-get-magnetic.8"
    )
endif()

