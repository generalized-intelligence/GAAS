# this keeps track of the targets that are generated for doxygen,
# used to set up the doc target later on
# unfortunately, this has to be a property instead of a variable due to cmake's
# weird scoping rules
set_property(GLOBAL PROPERTY _nvbio_doxygen_target_list "")

# set up doxygen for a given nvbio module
# checks if there's a Doxyfile.in in the module directory
# and sets up build rules to generate documentation for this module
macro(_nvbio_doxygen_module name)
  if (DOXYGEN_FOUND)
    # set up paths
    set(doxy_in_directory "${CMAKE_CURRENT_SOURCE_DIR}/doxy")
    set(doxy_out_directory "${CMAKE_CURRENT_BINARY_DIR}/doxy")
    # set up the path to the Doxyfile.in for the current module
    set(doxyfile_in "${doxy_in_directory}/Doxyfile.in")
    # set up the path to the Doxyfile that we'll generate from Doxyfile.in
    set(doxyfile_out "${doxy_out_directory}/Doxyfile")

    # test if the current module has a Doxyfile.in
    if (EXISTS ${doxyfile_in})
      message("==> Found Doxygen file for module ${name}")
      # parse the Doxyfile.in with cmake to substitute build variables and generate the Doxyfile
      configure_file(${doxyfile_in} ${doxyfile_out} @ONLY)
      add_custom_target(doc_${name}_copy
                        ${CMAKE_COMMAND} -E copy_directory ${doxy_in_directory} ${doxy_out_directory})
      add_custom_target(doc_${name}
                        DEPENDS doc_${name}_copy
                        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                        WORKING_DIRECTORY ${doxy_out_directory}
                        COMMENT "Generating API documentation for ${name} with Doxygen" VERBATIM)
      set_property(GLOBAL APPEND PROPERTY _nvbio_doxygen_target_list doc_${name})
    endif()
  endif()
endmacro(_nvbio_doxygen_module)

# generate the doc target
# doc will depend on all doc_* targets generated for any modules with doxygen.in files
macro(nvbio_doxygen)
  if (DOXYGEN_FOUND)
    get_property(_targets GLOBAL PROPERTY _nvbio_doxygen_target_list)
    add_custom_target(doc DEPENDS ${_targets})
  endif()
endmacro(nvbio_doxygen)

# start a new nvbio module (library or executable)
# this will match a MSVC project
macro(nvbio_module name)
  set(_current_nvbio_module ${name})
  set(_current_nvbio_module_srcs "")
  set(_current_nvbio_directory "")

  _nvbio_doxygen_module(${name})
endmacro(nvbio_module)

# add sources to the current module or directory
macro(addsources)
  set(_src_list_TMP "")
  # add the directory path
  foreach(l ${ARGN})
    list(APPEND _src_list_TMP ${_current_nvbio_directory}${l})
  endforeach()

  if (NOT "${_current_nvbio_directory}" STREQUAL "")
      # chop the trailing '/' off the group name
      string(REGEX REPLACE "/$" "" _tmp_group ${_current_nvbio_directory})
      # convert '/' into '\\'
      string(REGEX REPLACE "/" "\\\\" _tmp_group ${_tmp_group})
      source_group(${_tmp_group} FILES ${_src_list_TMP})
      unset(_tmp_group)
  else()
      source_group("" FILES ${_src_list_TMP})
  endif()

  list(APPEND ${_current_nvbio_module}_srcs ${_src_list_TMP})
  unset(_src_list_TMP)
endmacro(addsources)

# add a module directory
macro(nvbio_add_module_directory directory)
  set(_current_nvbio_directory "${directory}/")
  include(${directory}/CMakeLists.txt)
endmacro(nvbio_add_module_directory)

