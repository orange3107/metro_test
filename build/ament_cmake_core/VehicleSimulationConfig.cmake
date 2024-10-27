# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_VehicleSimulation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED VehicleSimulation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(VehicleSimulation_FOUND FALSE)
  elseif(NOT VehicleSimulation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(VehicleSimulation_FOUND FALSE)
  endif()
  return()
endif()
set(_VehicleSimulation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT VehicleSimulation_FIND_QUIETLY)
  message(STATUS "Found VehicleSimulation:  (${VehicleSimulation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'VehicleSimulation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${VehicleSimulation_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(VehicleSimulation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${VehicleSimulation_DIR}/${_extra}")
endforeach()
