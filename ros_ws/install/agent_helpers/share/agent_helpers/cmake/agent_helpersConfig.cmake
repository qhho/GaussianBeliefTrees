# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_agent_helpers_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED agent_helpers_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(agent_helpers_FOUND FALSE)
  elseif(NOT agent_helpers_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(agent_helpers_FOUND FALSE)
  endif()
  return()
endif()
set(_agent_helpers_CONFIG_INCLUDED TRUE)

# output package information
if(NOT agent_helpers_FIND_QUIETLY)
  message(STATUS "Found agent_helpers: 0.1.0 (${agent_helpers_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'agent_helpers' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${agent_helpers_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(agent_helpers_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${agent_helpers_DIR}/${_extra}")
endforeach()
