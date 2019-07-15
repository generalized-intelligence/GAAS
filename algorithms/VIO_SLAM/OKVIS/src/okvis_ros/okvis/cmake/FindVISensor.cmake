find_package(PkgConfig)
pkg_check_modules(PC_VISENSOR visensor)
#message("pkg: ${PC_VISENSOR_INCLUDEDIRS}")
set(VISENSOR_DEFINITIONS ${PC_VISENSOR_CFLAGS_OTHER})

find_path(VISensorDriver_INCLUDE_DIR visensor.hpp
          HINTS ${PC_VISENSOR_INCLUDEDIR} ${PC_VISENSOR_INCLUDE_DIRS}
          PATH_SUFFIXES visensor )

find_library(VISensorDriver_LIBRARY NAMES libvisensor.so
             HINTS ${PC_VISENSOR_LIBDIR} ${PC_VISENSOR_LIBRARY_DIRS} /usr/local/lib)

set(VISENSOR_LIBRARIES ${VISENSOR_LIBRARY} )
set(VISENSOR_INCLUDE_DIRS ${VISENSOR_INCLUDE_DIR} )


include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(VISensorDriver  DEFAULT_MSG
                                  VISensorDriver_LIBRARY VISensorDriver_INCLUDE_DIR)

mark_as_advanced(VISensorDriver_INCLUDE_DIR VISensorDriver_LIBRARY )
