# Install script for directory: /Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Accumulator.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/AlbersEqualArea.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/AzimuthalEquidistant.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/CassiniSoldner.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/CircularEngine.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Constants.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/DMS.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Ellipsoid.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/EllipticFunction.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Geocentric.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/GeoCoords.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Geodesic.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/GeodesicExact.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/GeodesicLine.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/GeodesicLineExact.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Geohash.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Geoid.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Gnomonic.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/GravityCircle.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/GravityModel.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/LambertConformalConic.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/LocalCartesian.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/MagneticCircle.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/MagneticModel.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Math.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/MGRS.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/NormalGravity.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/OSGB.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/PolarStereographic.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/PolygonArea.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Rhumb.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/SphericalEngine.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/SphericalHarmonic.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/SphericalHarmonic1.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/SphericalHarmonic2.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/TransverseMercator.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/TransverseMercatorExact.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/Utility.hpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/include/GeographicLib/UTMUPS.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES "/Users/Maverick/Documents/GeographicLib-1.42/build/include/GeographicLib/Config.h")
endif()

