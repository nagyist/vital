# Installation logic for MAP-Tk CMake utilities
#
# Variables that modify function:
#
#   kwiver_cmake_install_dir
#     - Directory to install files to
#
set(src_dir "${CMAKE_CURRENT_LIST_DIR}")

install(
  FILES "${src_dir}/kwiver-utils.cmake"
        "${src_dir}/FindEigen3.cmake"
        "${src_dir}/FindLog4cxx.cmake"
        "${src_dir}/vital-flags.cmake"
        "${src_dir}/vital-flags-gnu.cmake"
        "${src_dir}/vital-flags-msvc.cmake"
        "${src_dir}/vital-flags-clang.cmake"
        "${src_dir}/kwiver-configcheck.cmake"
        "${src_dir}/CommonFindMacros.cmake"
  DESTINATION "${kwiver_cmake_install_dir}"
  )

install(
  DIRECTORY "${src_dir}/utils"
            "${src_dir}/tools"
            "${src_dir}/configcheck"
  DESTINATION "${kwiver_cmake_install_dir}"
  )
