#1--------------------------->
cmake_minimum_required(VERSION 3.2)
project(DosuRay)
add_definitions(-std=c++11)                                          #specify compiler

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

#2--------------------------->
#eigen3
set(EIGEN_I /usr/include/eigen3)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${ASTRA_ss}/cmake")     #module-method to link libxxx.so, not useful here.
#PCL
find_package(PCL REQUIRED)
#OpenCV
find_package( OpenCV REQUIRED )
#BOOST
set(BOOST_LIBS program_options serialization random system)
find_package(Boost 1.48.0 COMPONENTS ${BOOST_LIBS} REQUIRED)
MESSAGE("Boost information:") 
MESSAGE("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
MESSAGE("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
MESSAGE("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")

#find_package(SFML COMPONENTS graphics window system)

include_directories(${EIGEN_I} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_library(RBTREE ${SOURCE_PATH}/Rbtree.cpp)
#add_library(CircleList ${SOURCE_PATH}/CircleLinkedList.cpp)
#add_library(Common_lib ${SOURCE_PATH}/Common.cpp)
#add_library(json_lib ${SOURCE_PATH}/jsoncpp.cpp)
# Sources files
set(DosuRay_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/Rbtree.cpp
  ${PROJECT_SOURCE_DIR}/src/CircleLinkedList.cpp
  ${PROJECT_SOURCE_DIR}/src/jsoncpp.cpp
  ${PROJECT_SOURCE_DIR}/src/FrameShell.cpp
  ${PROJECT_SOURCE_DIR}/src/Common.cpp
)

add_library(DosuRay_lib ${DosuRay_SOURCE_FILES})
target_link_libraries(DosuRay_lib
		      ${catkin_LIBRARIES} 
 		      ${PCL_LIBRARIES}
		      ${OpenCV_LIBS}
                      ${Boost_LIBRARIES})

#3--------------------------->
add_executable(DosuRay ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(DosuRay 
  DosuRay_lib
#  ${PCL_LIBRARIES}
#  ${OpenCV_LIBS}
#${Boost_LIBRARIES}
)

add_executable(DosuRay_test ${PROJECT_SOURCE_DIR}/src/test.cpp)
target_link_libraries(DosuRay_test 
 DosuRay_lib
#  ${PCL_LIBRARIES} 
#  ${OpenCV_LIBS}
#${Boost_LIBRARIES}
)

add_executable(DosuRay_6d_estimate ${PROJECT_SOURCE_DIR}/src/main_6d_estimate.cpp)
target_link_libraries(DosuRay_6d_estimate
 DosuRay_lib
#  ${PCL_LIBRARIES}
#  ${OpenCV_LIBS}
#${Boost_LIBRARIES}
)
