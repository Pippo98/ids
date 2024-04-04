message("[CORE]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR}/bin/core)

include_directories(
  ${EXTERNAL_DIRS}
  ${CDIR}/src
)

add_executable(prj ${CDIR}/executables/prj/main.cpp)