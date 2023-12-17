# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hyrian_track_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hyrian_track_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hyrian_track_FOUND FALSE)
  elseif(NOT hyrian_track_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hyrian_track_FOUND FALSE)
  endif()
  return()
endif()
set(_hyrian_track_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hyrian_track_FIND_QUIETLY)
  message(STATUS "Found hyrian_track: 0.0.0 (${hyrian_track_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hyrian_track' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${hyrian_track_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hyrian_track_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${hyrian_track_DIR}/${_extra}")
endforeach()
