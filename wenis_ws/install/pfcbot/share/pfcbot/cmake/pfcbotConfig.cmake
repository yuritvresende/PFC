# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pfcbot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pfcbot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pfcbot_FOUND FALSE)
  elseif(NOT pfcbot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pfcbot_FOUND FALSE)
  endif()
  return()
endif()
set(_pfcbot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pfcbot_FIND_QUIETLY)
  message(STATUS "Found pfcbot: 0.0.0 (${pfcbot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pfcbot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pfcbot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pfcbot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pfcbot_DIR}/${_extra}")
endforeach()
