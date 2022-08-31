cmake_minimum_required (VERSION 2.8.12)
project (MINAMI C CXX)

find_package(Threads)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_COMPILER g++)
set (CMAKE_CXX_FLAGS "-std=c++11 -m64 -Wall")
set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
# set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

#set (CMAKE_CXX_FLAGS "${CMAKE CXX FLAGS} −pg")
#set (CMAKE_EXE_LIKKER_FLAGS "-pthread")

# https://stackoverflow.com/questions/24174394/cmake-is-not-able-to-find-python-libraries
# set (PYTHON_EXECUTABLE:FILEPATH "/home/qiling/anaconda3/envs/mac_posts_python3/bin/")
# set (PYTHON_INCLUDE_DIR "/home/qiling/anaconda3/envs/mac_posts_python3/include/python3.9")
# set (PYTHON_LIBRARY "/home/qiling/anaconda3/envs/mac_posts_python3/lib/libpython3.9.so")
# set (PYTHON_INCLUDE_DIR "$(python -c \"from distutils.sysconfig import get_python_inc; print(get_python_inc())\")")
# set (PYTHON_LIBRARY "$(python -c \"import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))\")")

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


