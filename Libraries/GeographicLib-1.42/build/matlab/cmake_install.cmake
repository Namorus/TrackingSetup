# Install script for directory: /Users/Maverick/Documents/GeographicLib-1.42/matlab

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/matlab/geographiclib" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/cassini_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/cassini_inv.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/Contents.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/defaultellipsoid.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/ecc2flat.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/eqdazim_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/eqdazim_inv.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/flat2ecc.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/gedistance.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/gedoc.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/geocent_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/geocent_inv.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/geodarea.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/geoddistance.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/geoddoc.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/geodreckon.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/geoid_height.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/geoid_load.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/gereckon.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/gnomonic_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/gnomonic_inv.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/loccart_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/loccart_inv.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/mgrs_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/mgrs_inv.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/polarst_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/polarst_inv.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/projdoc.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/tranmerc_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/tranmerc_inv.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/utmups_fwd.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/utmups_inv.m"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/matlab/geographiclib/private" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/A1m1f.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/A2m1f.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/A3coeff.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/A3f.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/AngDiff.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/AngNormalize.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/AngNormalize2.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/AngRound.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/C1f.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/C1pf.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/C2f.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/C3coeff.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/C3f.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/C4coeff.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/C4f.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/cbrtx.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/cvmgt.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/eatanhe.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/G4coeff.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/geoid_file.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/geoid_load_file.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/GeoRotation.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/norm2.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/SinCosSeries.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/sumx.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/swap.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/tauf.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib/private/taupf.m"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/matlab/geographiclib-legacy" TYPE FILE FILES
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/Contents.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geocentricforward.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geocentricreverse.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geodesicdirect.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geodesicinverse.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geodesicline.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geographiclibinterface.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geoidheight.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/localcartesianforward.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/localcartesianreverse.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/mgrsforward.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/mgrsreverse.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/polygonarea.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/utmupsforward.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/utmupsreverse.m"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geocentricforward.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geocentricreverse.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geodesicdirect.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geodesicinverse.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geodesicline.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/geoidheight.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/localcartesianforward.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/localcartesianreverse.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/mgrsforward.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/mgrsreverse.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/polygonarea.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/utmupsforward.cpp"
    "/Users/Maverick/Documents/GeographicLib-1.42/matlab/geographiclib-legacy/utmupsreverse.cpp"
    )
endif()

