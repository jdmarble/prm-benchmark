find_library(ASSIMP_LIBRARY assimp DOC "Location of 3D asset importer library")
find_path(ASSIMP_INCLUDE_DIR assimp.h PATH_SUFFIXES assimp
   DOC "Location of 3D asset importer header file directory")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(assimp DEFAULT_MSG ASSIMP_LIBRARY ASSIMP_INCLUDE_DIR)
