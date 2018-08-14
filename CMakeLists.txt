project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance

add_definitions(-std=c++11)
# this only for Alex's problematic Mac
# add_definitions(-std=c++11 -fopenmp)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

# this only for Alex's problematic Mac
# SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,/usr/local/opt/llvm/lib")

set(sources src/MPC.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# this only for Alex's problematic Mac
# link_directories(/usr/local/opt/llvm/lib)
# include_directories(/usr/local/opt/llvm/include)

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)
# this only for Alex's problematic Mac
# target_link_libraries(mpc ipopt z ssl uv uWS -fopenmp)

