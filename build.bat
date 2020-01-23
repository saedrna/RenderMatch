mkdir build
cd build

set UNIX_PREFIX=%CONDA_PREFIX:%
set UNIX_LIBRARY_PREFIX=%LIBRARY_PREFIX:%
set UNIX_LIBRARY_BIN=%LIBRARY_BIN:%
set UNIX_LIBRARY_INC=%LIBRARY_INC:%
set UNIX_LIBRARY_LIB=%LIBRARY_LIB:%
set UNIX_SP_DIR=%SP_DIR:%
set UNIX_SRC_DIR=%SRC_DIR:%

pushd C:\Program Files (x86)\Microsoft Visual Studio\2017\Community
call VC\Auxiliary\Build\vcvars64.bat
popd

cmake -G "Visual Studio 15 2017" -A "x64" -T "host=x64" ^
    -DCMAKE_TOOLCHAIN_FILE=%UNIX_PREFIX%h2o.cmake ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_INSTALL_PREFIX=./install ^
    -DBUILD_SHARED_LIBS=ON ^
    ../
