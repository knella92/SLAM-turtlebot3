# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nuslam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nuslam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nuslam_FOUND FALSE)
  elseif(NOT nuslam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nuslam_FOUND FALSE)
  endif()
  return()
endif()
set(_nuslam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nuslam_FIND_QUIETLY)
  message(STATUS "Found nuslam: 0.0.0 (${nuslam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nuslam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${nuslam_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nuslam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nuslam_DIR}/${_extra}")
endforeach()
