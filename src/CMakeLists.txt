cmake_minimum_required(VERSION 2.8.12)
project(MY_BZZKH4)
set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_INSTALL_PREFIX}/share/buzz/cmake)

find_package(Buzz REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIRS} )
message("Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

include_directories(${CMAKE_SOURCE_DIR}/buzz ${BUZZ_C_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/buzz ${KHEPERA_INCLUDE_DIR} ${BUZZ_C_INCLUDE_DIR})
link_directories(${KHEPERA_LIBRARY_DIR})

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
  buzz/include/alglib/stdafx.h
  buzz/bzzcognifly.c)
target_link_libraries(bzzCognifly PUBLIC ${EIGEN3_LIBRARY} ${BUZZ_LIBRARY} ${BUZZ_LIBRARY_DEBUG} -lpthread -lm)

install(TARGETS bzzCognifly RUNTIME DESTINATION bin)
