cmake_minimum_required(VERSION 3.1)

project(PcbRouter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(PCBROUTER_HOME ${PROJECT_SOURCE_DIR} )
set(KICADPCB_HOME
  ${PROJECT_SOURCE_DIR}/module/kicadPcb
)

add_subdirectory(${KICADPCB_HOME})


set (PCBROUTER_SRC 
  src/BoardGrid.cpp
  src/GridBasedRouter.cpp
  src/plotter.cpp
  src/globalParam.cpp
  )

set (PCBROUTER_HEADER
  src/BoardGrid.h
  src/GridBasedRouter.h
  src/plotter.h
  src/globalParam.h
  src/util.h
  )

############################################################
# include directories 
############################################################
include_directories(
  ${KICADPCB_HOME}/src
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

add_executable( pcbrouter src/main.cpp )

add_dependencies( pcbrouter kicadpcbparserlib )

target_link_libraries( pcbrouter
  pcbrouterlib
  kicadpcbparserlib
)

############################################################
# Install  
############################################################

install(FILES ${PCBROUTER_HEADER} DESTINATION include)
install(TARGETS pcbrouter DESTINATION bin)
install(TARGETS pcbrouterlib DESTINATION lib)

