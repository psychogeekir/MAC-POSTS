cmake_minimum_required (VERSION 2.8)
project (MINAMI C CXX)

find_package(Threads)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_COMPILER g++)
set (CMAKE_CXX_FLAGS "-std=c++11 -m64 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

#set (CMAKE_CXX_FLAGS "${CMAKE CXX FLAGS} −pg")
#set (CMAKE_EXE_LIKKER_FLAGS "-pthread")

if (APPLE)
    set(CMAKE_MACOSX_RPATH ON)
endif (APPLE)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# generate libGlib.so
add_subdirectory (glib-core)
# generate libSnap.so
add_subdirectory (snap-core)
# generate libadv_ds.so and libg3log.so
add_subdirectory (3rdparty)
# generate libminami.so
add_subdirectory (minami)
add_subdirectory (examples)
# generate MNMAPI.so
add_subdirectory (pybinder)


