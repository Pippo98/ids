message("[CORE]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR}/bin/core)

include_directories(
  ${EXTERNAL_DIRS}
  ${CDIR}/src
)

add_library(agent)
target_sources(
  agent
  PRIVATE
  ${CDIR}/src/agent/Agent.cpp
  )
target_link_libraries(agent raylib )

add_library(communication)
target_sources(
  communication
  PRIVATE
  ${CDIR}/src/communication/Broker.cpp
  )
target_link_libraries(communication raylib agent)

add_library(world)
target_sources(
  world
  PRIVATE
  ${CDIR}/src/world/World.cpp
  )
target_link_libraries(world raylib agent communication)

add_executable(prj ${CDIR}/executables/prj/main.cpp)
target_link_libraries(prj PUBLIC raylib agent)
