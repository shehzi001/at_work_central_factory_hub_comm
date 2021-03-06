cmake_minimum_required(VERSION 2.8.7)
project(protobuf_comm)

find_package(Boost 1.46.1 REQUIRED thread system)
find_package(OpenSSL REQUIRED)

set(CFH_ROOT_DIR ${PROJECT_SOURCE_DIR}/../at_work_central_factory_hub)
set(PROTOBUF_COMM_SRC_DIR ${CFH_ROOT_DIR}/src/libs/protobuf_comm)
set(CFH_LIBS_DIR ${CFH_ROOT_DIR}/src/libs)

link_directories(${Boost_LIBRARY_DIRS})

add_definitions("-std=c++0x")
add_definitions("-DHAVE_LIBCRYPTO")

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
  ${CFH_LIBS_DIR}
)

file(GLOB PROTOBUF_COMM_SOURCES ${PROTOBUF_COMM_SRC_DIR}/*.cpp)
file(GLOB PROTOBUF_COMM_HEADERS ${PROTOBUF_COMM_SRC_DIR}/*.h)

add_library(protobuf_comm SHARED ${PROTOBUF_COMM_SOURCES})

target_link_libraries(protobuf_comm ${OPENSSL_LIBRARIES} ${Boost_LIBRARIES})

## if: using catkin_make
if(DEFINED _CATKIN_ALL_INCLUDED_)
  find_package(catkin REQUIRED)

  catkin_package(
    LIBRARIES
      protobuf_comm
  )

  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME} )

  add_custom_target(${PROJECT_NAME}_copy_headers ALL)

  foreach(File ${PROTOBUF_COMM_HEADERS})
    add_custom_command(TARGET ${PROJECT_NAME}_copy_headers POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy ${File} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
    )
  endforeach()

  install(FILES ${PROTOBUF_COMM_HEADERS}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
## else: using cmake 
else()
  install(
    TARGETS protobuf_comm
    LIBRARY DESTINATION lib
  )
  install(
    FILES ${PROTOBUF_COMM_HEADERS}
    DESTINATION include/${PROJECT_NAME}
  )

endif(DEFINED _CATKIN_ALL_INCLUDED_)

