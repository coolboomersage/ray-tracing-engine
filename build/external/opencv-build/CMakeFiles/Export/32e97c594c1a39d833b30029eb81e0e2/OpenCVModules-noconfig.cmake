#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "opencv_core" for configuration ""
set_property(TARGET opencv_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(opencv_core PROPERTIES
  IMPORTED_IMPLIB_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_core500.dll.a"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_core500.dll"
  )

list(APPEND _cmake_import_check_targets opencv_core )
list(APPEND _cmake_import_check_files_for_opencv_core "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_core500.dll.a" "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_core500.dll" )

# Import target "opencv_flann" for configuration ""
set_property(TARGET opencv_flann APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(opencv_flann PROPERTIES
  IMPORTED_IMPLIB_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_flann500.dll.a"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_flann500.dll"
  )

list(APPEND _cmake_import_check_targets opencv_flann )
list(APPEND _cmake_import_check_files_for_opencv_flann "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_flann500.dll.a" "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_flann500.dll" )

# Import target "opencv_geometry" for configuration ""
set_property(TARGET opencv_geometry APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(opencv_geometry PROPERTIES
  IMPORTED_IMPLIB_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_geometry500.dll.a"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_geometry500.dll"
  )

list(APPEND _cmake_import_check_targets opencv_geometry )
list(APPEND _cmake_import_check_files_for_opencv_geometry "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_geometry500.dll.a" "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_geometry500.dll" )

# Import target "opencv_imgproc" for configuration ""
set_property(TARGET opencv_imgproc APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(opencv_imgproc PROPERTIES
  IMPORTED_IMPLIB_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_imgproc500.dll.a"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_imgproc500.dll"
  )

list(APPEND _cmake_import_check_targets opencv_imgproc )
list(APPEND _cmake_import_check_files_for_opencv_imgproc "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_imgproc500.dll.a" "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_imgproc500.dll" )

# Import target "opencv_imgcodecs" for configuration ""
set_property(TARGET opencv_imgcodecs APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(opencv_imgcodecs PROPERTIES
  IMPORTED_IMPLIB_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_imgcodecs500.dll.a"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "OpenEXR::OpenEXR"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_imgcodecs500.dll"
  )

list(APPEND _cmake_import_check_targets opencv_imgcodecs )
list(APPEND _cmake_import_check_files_for_opencv_imgcodecs "${_IMPORT_PREFIX}/x64/mingw/lib/libopencv_imgcodecs500.dll.a" "${_IMPORT_PREFIX}/x64/mingw/bin/libopencv_imgcodecs500.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
