project( boost_program_options_example )
cmake_minimum_required( VERSION 3.0 )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

find_package( Boost COMPONENTS program_options REQUIRED )

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR} )

add_executable( ${PROJECT_NAME} main.cc )
target_link_libraries( ${PROJECT_NAME}
    ${Boost_LIBRARIES} )

