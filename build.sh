cd build
echo "$0"
echo "$1"
# assume swith to correct python env in anaconda
# https://stackoverflow.com/questions/72502292/fatal-error-python-h-no-such-file-or-directory-when-compiling-pybind11-example
# https://stackoverflow.com/questions/24174394/cmake-is-not-able-to-find-python-libraries
# check generated CMakeCache.txt
if [ $1 = "Release" ]; then
  echo "Build Release version"
  cmake -DCMAKE_BUILD_TYPE=$1 ../src \
        -DCMAKE_CXX_FLAGS="-std=c++11 -m64 -Wall -O2" \
        -DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")  \
        -DPYTHON_LIBRARY=$(python -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))") \
        -DPYTHON_EXECUTABLE:FILEPATH=`which python`
else
  echo "Build Debug version"
  cmake ../src \
        -DCMAKE_CXX_FLAGS="-std=c++11 -m64 -Wall" \
        -DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")  \
        -DPYTHON_LIBRARY=$(python -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))") \
        -DPYTHON_EXECUTABLE:FILEPATH=`which python`
fi
