message("[EXTERNAL]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})

set(EXTERNAL_DIRS
  ${CDIR}
  ${CDIR}/raylib
)
include_directories(PUBLIC ${EXTERNAL_DIRS})
set(EXTERNAL_DIRS ${EXTERNAL_DIRS} PARENT_SCOPE)

# raylib
add_subdirectory(raylib)

add_subdirectory(eigen)
