cmake_minimum_required(VERSION 3.5)
project(MY_COGNIFLY LANGUAGES C CXX)
set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_INSTALL_PREFIX}/share/buzz/cmake)

find_package(Buzz REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIRS} )
message("Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set (CMAKE_PREFIX_PATH "/usr/include/")
# target_include_directories(asio INTERFACE /usr/include/asio)
find_package(ASIO REQUIRED)
add_definitions(-DASIO_STANDALONE)
add_definitions(-DASIO_HAS_STD_ADDRESSOF)
add_definitions(-DASIO_HAS_STD_ARRAY)
add_definitions(-DASIO_HAS_CSTDINT)
add_definitions(-DASIO_HAS_STD_SHARED_PTR)
add_definitions(-DASIO_HAS_STD_TYPE_TRAITS)

set(MSP_SOURCE_DIR buzz/src/src_fc)
set(MSP_INCLUDE_DIR buzz/include/inc_fc/msp)
include_directories(${MSP_INCLUDE_DIR})



include_directories(${CMAKE_SOURCE_DIR}/buzz ${BUZZ_C_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/buzz ${KHEPERA_INCLUDE_DIR} ${BUZZ_C_INCLUDE_DIR})
link_directories(${KHEPERA_LIBRARY_DIR})

# client library
add_library(mspclient ${MSP_SOURCE_DIR}/Client.cpp ${MSP_SOURCE_DIR}/PeriodicTimer.cpp)
target_link_libraries(mspclient ${CMAKE_THREAD_LIBS_INIT} ASIO::ASIO)

add_executable(bzzCognifly
  buzz/buzz_utility.c buzz/buzz_utility.h
  buzz/buzzcognifly_closures.c buzz/buzzcognifly_closures.h
  buzz/cognifly_utility.c buzz/cognifly_utility.h
  buzz/intermediate.cpp				buzz/intermediate.h
  buzz/src/QPSolver.cpp                         buzz/include/QPSolver.h
  buzz/src/lambda_2.cpp				buzz/include/lambda_2.h
  buzz/include/alglib/alglibinternal.cpp        buzz/include/alglib/alglibinternal.h
  buzz/include/alglib/alglibmisc.cpp            buzz/include/alglib/alglibmisc.h
  buzz/include/alglib/ap.cpp                    buzz/include/alglib/ap.h
  buzz/include/alglib/linalg.cpp                buzz/include/alglib/linalg.h
  buzz/include/alglib/optimization.cpp          buzz/include/alglib/optimization.h
  buzz/include/alglib/solvers.cpp               buzz/include/alglib/solvers.h
  buzz/include/inc_fc/msp/ByteVector.hpp        buzz/include/inc_fc/msp/FlightController.hpp
  buzz/include/inc_fc/msp/msp_msg.hpp           buzz/include/inc_fc/msp/Value.hpp
  buzz/include/inc_fc/msp/Client.hpp            buzz/include/inc_fc/msp/FlightMode.hpp
  buzz/include/inc_fc/msp/PeriodicTimer.hpp     buzz/include/inc_fc/msp/FirmwareVariants.hpp
  buzz/include/inc_fc/msp/Message.hpp           buzz/include/inc_fc/msp/Subscription.hpp
  buzz/include/alglib/stdafx.h                  buzz/src/src_fc/Client.cpp
  buzz/src/src_fc/FlightController.cpp          buzz/src/src_fc/PeriodicTimer.cpp
  buzz/fc_inav.cpp                              buzz/fc_inav.h
  buzz/bzzcognifly.c)
target_link_libraries(bzzCognifly PUBLIC ${EIGEN3_LIBRARY} ${BUZZ_LIBRARY} ${BUZZ_LIBRARY_DEBUG} -lpthread -lm)

install(TARGETS bzzCognifly RUNTIME DESTINATION bin)
