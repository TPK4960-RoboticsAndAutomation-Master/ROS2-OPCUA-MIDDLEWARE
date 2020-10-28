# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_kmr_simulation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED kmr_simulation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(kmr_simulation_FOUND FALSE)
  elseif(NOT kmr_simulation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(kmr_simulation_FOUND FALSE)
  endif()
  return()
endif()
set(_kmr_simulation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT kmr_simulation_FIND_QUIETLY)
  message(STATUS "Found kmr_simulation: 0.0.0 (${kmr_simulation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'kmr_simulation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${kmr_simulation_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(kmr_simulation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${kmr_simulation_DIR}/${_extra}")
endforeach()
