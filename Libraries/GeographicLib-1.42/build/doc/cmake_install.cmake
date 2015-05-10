# Install script for directory: /Users/Maverick/Documents/GeographicLib-1.42/doc

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/GeographicLib/html" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/build/doc/html/LICENSE.txt"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/doc/html/index.html"
    "/Users/Maverick/Documents/GeographicLib-1.42/build/doc/html/utilities.html"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/GeographicLib/scripts" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/geod-calc.html"
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/geod-google-instructions.html"
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/geod-google.html"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/GeographicLib/scripts/GeographicLib" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/GeographicLib/DMS.js"
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/GeographicLib/Geodesic.js"
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/GeographicLib/GeodesicLine.js"
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/GeographicLib/Interface.js"
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/GeographicLib/Math.js"
    "/Users/Maverick/Documents/GeographicLib-1.42/doc/scripts/GeographicLib/PolygonArea.js"
    )
endif()

