cmake_minimum_required (VERSION 2.6.0)
project(mcsimplugin)
find_package(OpenRAVE 0.9 REQUIRED)

# C++ 11 support
set (CMAKE_CXX_STANDARD 11)
#Armadillo library
find_package(Armadillo REQUIRED)

#add_definitions("-DARMA_DONT_USE_WRAPPER -lopenblas -llapack -lhdf5")

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(${ARMADILLO_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(mcsimplugin SHARED mcsimplugin.cpp)
set_target_properties(mcsimplugin PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(mcsimplugin ${OpenRAVE_LIBRARIES})
target_link_libraries(mcsimplugin ${ARMADILLO_LIBRARIES})
install(TARGETS mcsimplugin DESTINATION .)

