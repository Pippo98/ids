message("[EXTERNAL]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})

set(EXTERNAL_DIRS
  ${CDIR}
  ${CDIR}/raylib
  ${CDIR}/kflib
  ${CDIR}/kflib/eigen
)
include_directories(PUBLIC ${EXTERNAL_DIRS})
set(EXTERNAL_DIRS ${EXTERNAL_DIRS} PARENT_SCOPE)

# raylib
add_subdirectory(raylib)
set(BUILD_SHARED_LIBS ON)
add_subdirectory(kflib)
