cmake_minimum_required(VERSION 3.1)

project(PcbRouter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(PCBROUTER_HOME ${PROJECT_SOURCE_DIR} )
set(KICADPCB_HOME
  ${PROJECT_SOURCE_DIR}/module/kicadPcb
)

add_subdirectory(${KICADPCB_HOME})

# set(ENV{TYPE_CALL} library)

## configure settings
#find_package(GoogleHash)
#find_package(MKL REQUIRED)
#find_package(TCL REQUIRED)
#find_package(Boost REQUIRED)
#find_package(Threads REQUIRED)

set (PCBROUTER_SRC 
  src/BoardGrid.cpp
#  src/kicadPcbDataBase.cpp
  src/main.cpp
  )

set (PCBROUTER_HEADER
  src/BoardGrid.h
#  src/kicadPcbDataBase.h
#  src/mymath.h
#  src/router.h
  )

############################################################
# include directories 
############################################################
include_directories(
  ${KICADPCB_HOME}/src

  ${X11_INCLUDE_DIR}
  ${MKL_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}

  ${EIGEN_HOME}
  ${CIMG_HOME}

  ${DEFLIB_HOME}/def
  ${DEFLIB_HOME}/defzlib
  ${DEFLIB_HOME}/cdef
  ${DEFLIB_HOME}/cdefzlib

  ${LEFLIB_HOME}/lef
  ${LEFLIB_HOME}/lefzlib
  ${LEFLIB_HOME}/clef
  ${LEFLIB_HOME}/clefzlib

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
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PCBROUTER_HOME}/bin)

add_executable( pcbrouter
  ${PCBROUTER_SRC}
)

add_dependencies( pcbrouter kicadpcbparser )
#add_dependencies(  flute )
#add_dependencies( replace def )
#add_dependencies( replace lef )

#add_dependencies( replacelib sta )
#add_dependencies( replacelib flute )
#add_dependencies( replacelib def )
#add_dependencies( replacelib lef )

target_link_libraries( pcbrouter
  PUBLIC
  kicadpcbparserlib
)

############################################################
# Install  
############################################################

install(FILES ${PCBROUTER_HEADER} DESTINATION include)
install(TARGETS pcbrouter DESTINATION bin)
install(TARGETS pcbrouterlib DESTINATION lib)

