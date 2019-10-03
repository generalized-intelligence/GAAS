# -----------------------------------------------
# File that provides "make uninstall" target
#  We use the file 'install_manifest.txt'
# -----------------------------------------------
IF(NOT EXISTS "/home/gishr/software/GAAS_master/GAAS/algorithms/loop_closing/DBow3/cmake-build-debug/install_manifest.txt")
  MESSAGE(FATAL_ERROR "Cannot find install manifest: \"/home/gishr/software/GAAS_master/GAAS/algorithms/loop_closing/DBow3/cmake-build-debug/install_manifest.txt\"")
ENDIF(NOT EXISTS "/home/gishr/software/GAAS_master/GAAS/algorithms/loop_closing/DBow3/cmake-build-debug/install_manifest.txt")

FILE(READ "/home/gishr/software/GAAS_master/GAAS/algorithms/loop_closing/DBow3/cmake-build-debug/install_manifest.txt" files)
STRING(REGEX REPLACE "\n" ";" files "${files}")
FOREACH(file ${files})
  MESSAGE(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
#  IF(EXISTS "$ENV{DESTDIR}${file}")
#    EXEC_PROGRAM(
#      "/home/gishr/Downloads/CLion-2019.2.1/clion-2019.2.1/bin/cmake/linux/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
#      OUTPUT_VARIABLE rm_out
#      RETURN_VALUE rm_retval
#      )
	EXECUTE_PROCESS(COMMAND rm $ENV{DESTDIR}${file})
#    IF(NOT "${rm_retval}" STREQUAL 0)
#      MESSAGE(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
#    ENDIF(NOT "${rm_retval}" STREQUAL 0)
#  ELSE(EXISTS "$ENV{DESTDIR}${file}")
#    MESSAGE(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
#  ENDIF(EXISTS "$ENV{DESTDIR}${file}")
ENDFOREACH(file)


