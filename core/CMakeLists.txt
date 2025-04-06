message("[CORE]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR}/bin/core)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(
  ${EXTERNAL_DIRS}
  ${CDIR}/src
)

add_library(position
  ${CDIR}/src/position/PositionModel.cpp
  )
target_link_libraries(position raylib)
add_library(controller
  ${CDIR}/src/controller/ControlModel.cpp
  )
  target_link_libraries(controller PUBLIC position)


add_library(agent)
target_sources(
  agent
  PRIVATE
  ${CDIR}/src/agent/Agent.cpp
  )
  target_link_libraries(agent PUBLIC raylib kflib controller)

add_library(communication)
target_sources(
  communication
  PRIVATE
  ${CDIR}/src/communication/Broker.cpp
  ${CDIR}/src/communication/Message.cpp
  )
target_link_libraries(communication PUBLIC agent)

add_library(world)
target_sources(
  world
  PRIVATE
  ${CDIR}/src/world/World.cpp
  )
target_link_libraries(world PUBLIC agent communication)

add_library(geometry)
target_sources(
    geometry
    PRIVATE
    ${CDIR}/src/geometry/Geometry.cpp
)
target_link_libraries(geometry PUBLIC raylib)

add_library(voronoi)
target_sources(
    voronoi
    PRIVATE
    ${CDIR}/src/voronoi/Voronoi.cpp
)
target_link_libraries(voronoi PUBLIC raylib geometry)
add_library(map)
target_sources(
    map
    PRIVATE
    ${CDIR}/src/map/Map.cpp
)
target_link_libraries(map PUBLIC raylib)

add_executable(prj ${CDIR}/executables/prj/main.cpp)
target_link_libraries(prj PUBLIC agent communication voronoi map kflib)

add_executable(test ${CDIR}/executables/tests/main.cpp)
target_link_libraries(test PUBLIC agent communication voronoi map kflib)
