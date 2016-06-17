cmake_minimum_required(VERSION 2.8.3)

project(egomo-tools)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Torch REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)
#find_package(lz4 REQUIRED)

# TODO: Replace by a correct package finding routine
set(LIBLZ4_LIBRARIES "/usr/local/lib/liblz4.so" )
set(LIBLZ4_INCLUDE_DIRS "/usr/local/include" )


set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmax-errors=3 -Wall" )

set(src
  "${SOURCE_DIR}/lz4Compression.cpp"
  "${SOURCE_DIR}/reorder_bits.cpp"
)

include_directories(
  ${Torch_INSTALL_INCLUDE}
  ${catkin_INCLUDE_DIRS}
  ${LIBLZ4_INCLUDE_DIRS}
)

link_directories(
  ${Torch_INSTALL_LIB}
)

add_library(egomo-tools MODULE ${src})

target_link_libraries(egomo-tools TH ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBLZ4_LIBRARIES})

install(TARGETS egomo-tools LIBRARY DESTINATION ${Torch_INSTALL_LUA_CPATH_SUBDIR})
install(DIRECTORY "lua/" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/egomo-tools" FILES_MATCHING PATTERN "*.lua")
