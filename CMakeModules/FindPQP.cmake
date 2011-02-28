find_library(PQP_LIBRARY PQP DOC "Location of PQP proximity query library")
find_path(PQP_INCLUDE_DIR PQP.h PATH_SUFFIXES "PQP"
    DOC "Location of PQP proximity query header files")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pqp DEFAULT_MSG PQP_LIBRARY PQP_INCLUDE_DIR)
