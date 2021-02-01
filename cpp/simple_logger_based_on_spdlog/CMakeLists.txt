project( spdlog_demo )
cmake_minimum_required( VERSION 3.0 )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

# find_package( XXX REQUIRED )

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

# link_directories(
#     ${XXX_LIBRARY_DIRS} )

file( GLOB SRCS *.c *.cpp *.cc *.h *.hpp )
add_executable( ${PROJECT_NAME} ${SRCS} )

# target_link_libraries( ${PROJECT_NAME}
#     ${XXX_LIBRARIES} )
