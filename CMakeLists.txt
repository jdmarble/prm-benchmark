cmake_minimum_required(VERSION 2.6.2)
project(ompl-benchmark)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)
message("${PROJECT_SOURCE_DIR}/CMakeModules")

find_package(OMPL REQUIRED)
include_directories (${OMPL_INCLUDE_DIR})

find_package(OMPLAPP REQUIRED)
include_directories (${OMPLAPP_INCLUDE_DIR})

find_package(PQP REQUIRED)
include_directories (${PQP_INCLUDE_DIR})

find_package(ASSIMP REQUIRED)
include_directories (${ASSIMP_INCLUDE_DIR})

find_package(Boost REQUIRED program_options filesystem)
include_directories (${Boost_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${PROJECT_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
	${OMPL_LIBRARY}
	${OMPLAPP_LIBRARY}
	${PQP_LIBRARY}
	${ASSIMP_LIBRARY}
	${Boost_PROGRAM_OPTIONS_LIBRARY}
	)

add_executable(unf src/main_smooth.cpp)
target_link_libraries(unf
	${OMPL_LIBRARY}
	${OMPLAPP_LIBRARY}
	${PQP_LIBRARY}
	${ASSIMP_LIBRARY}
	${Boost_PROGRAM_OPTIONS_LIBRARY}
	)

add_executable(draw_graph src/main_graph.cpp)
target_link_libraries(draw_graph
	${OMPL_LIBRARY}
	${OMPLAPP_LIBRARY}
	${PQP_LIBRARY}
	${ASSIMP_LIBRARY}
	${Boost_LIBRARIES}
	)

#add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
