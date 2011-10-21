find_library(OMPLAPP_LIBRARY ompl_app DOC "Location of Open Motion Planning Library app")
find_path(OMPLAPP_INCLUDE_DIR omplapp/config.h PATH_SUFFIXES "OMPLAPP"
    DOC "Location of Open Motion Planning Library app header files")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(omplapp DEFAULT_MSG OMPLAPP_LIBRARY OMPLAPP_INCLUDE_DIR)
