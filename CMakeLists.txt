cmake_minimum_required(VERSION 3.1)

project(PcbRouter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(NOT CMAKE_BUILD_TYPE)
    # default to Release build for GCC builds
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release."
        FORCE)
endif()

set(PCBROUTER_HOME ${PROJECT_SOURCE_DIR} )
set(KICADPCB_HOME
  ${PROJECT_SOURCE_DIR}/module/kicadPcb
)

find_package(Boost REQUIRED)
#find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonInterp 3 REQUIRED)
#find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(SWIG REQUIRED)

find_library(SWIG_EXECUTABLE swig)
if(NOT SWIG_EXECUTABLE)
set(SWIG_EXECUTABLE "swig")
endif()
add_subdirectory(${KICADPCB_HOME})

set (PCBROUTER_SRC 
  src/BoardGrid.cpp
  src/GridBasedRouter.cpp
  src/GridNetclass.cpp
  src/GridPath.cpp
  src/MultipinRoute.cpp
  src/globalParam.cpp
  src/frTime.cpp
  src/frTime_helper.cpp
  src/PcbRouter_wrapper.cpp
  )

set (PCBROUTER_HEADER
  src/BoardGrid.h
  src/GridBasedRouter.h
  src/GridNetclass.h
  src/GridCell.h
  src/GridPin.h
  src/GridPath.h
  src/MultipinRoute.h
  src/IncrementalSearchGrids.h
  src/Location.h
  src/globalParam.h
  src/frTime.h
  src/util.h
  )

set (ROUTER_SWIG_FILES
  src/PcbRouter.i
)

############################################################
# include directories 
############################################################
include_directories(
  ${Boost_INCLUDE_DIR}
  ${KICADPCB_HOME}/src
  ${PYTHON_INCLUDE_DIRS}
  )

############################################################
# Create a library 
############################################################
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PCBROUTER_HOME}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PCBROUTER_HOME}/lib)

add_library( pcbrouterlib STATIC
  ${PCBROUTER_SRC}
)

set_target_properties( pcbrouterlib 
  PROPERTIES OUTPUT_NAME "pcbrouter"
)

############################################################
# Create an executable
############################################################

add_custom_command( OUTPUT ${PCBROUTER_HOME}/src/PcbRouter_wrapper.cpp ${PCBROUTER_LIB_FILES}
  COMMAND ${SWIG_EXECUTABLE} -python -py3 -c++ -o ${PCBROUTER_HOME}/src/PcbRouter_wrapper.cpp ${PCBROUTER_HOME}/src/PcbRouter.i
  WORKING_DIRECTORY ${PCBROUTER_HOME}
  DEPENDS ${ROUTER_SWIG_FILES}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PCBROUTER_HOME}/bin)

add_executable( pcbrouter src/main.cpp )

add_dependencies( pcbrouter kicadpcbparserlib )

target_link_libraries( pcbrouter
  pcbrouterlib
  kicadpcbparserlib
  ${PYTHON_LIBRARIES}
)

############################################################
# Install  
############################################################

install(FILES ${PCBROUTER_HEADER} DESTINATION include)
install(TARGETS pcbrouter DESTINATION bin)
install(TARGETS pcbrouterlib DESTINATION lib)
