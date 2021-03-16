project( pcl_pcd_to_xyz )
cmake_minimum_required( VERSION 3.0 )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

find_package( PCL REQUIRED )
find_package( Boost COMPONENTS program_options filesystem REQUIRED )

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR} )

link_directories(
    ${PCL_LIBRARY_DIRS} )

add_definitions( ${PCL_DEFINITIONS} )

file( GLOB SRCS *.c *.cpp *.cc *.h *.hpp )
add_executable( ${PROJECT_NAME} ${SRCS} )

target_link_libraries( ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} )
