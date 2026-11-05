cmake_minimum_required(VERSION 3.18)
project(libtprt VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Опции компиляции
if(MSVC)
    add_compile_options(/W4 /O2)
else()
    add_compile_options(-Wall -Wextra -O3 -march=native -fopenmp)
endif()

# 1. Поиск Eigen3
find_package(Eigen3 3.3 REQUIRED CONFIG)

# 2. Поиск RapidJSON (Header-only библиотека)
find_package(RapidJSON QUIET)
if(RAPIDJSON_INCLUDE_DIRS)
    include_directories(${RAPIDJSON_INCLUDE_DIRS})
elseif(RapidJSON_INCLUDE_DIRS)
    include_directories(${RapidJSON_INCLUDE_DIRS})
endif()

# 3. Поиск NLopt с автоопределением цели
find_package(NLopt QUIET)
if(TARGET NLopt::nlopt)
    set(NLOPT_LIB NLopt::nlopt)
elseif(TARGET nlopt::nlopt)
    set(NLOPT_LIB nlopt::nlopt)
elseif(TARGET nlopt)
    set(NLOPT_LIB nlopt)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(NLOPT REQUIRED nlopt)
    add_library(nlopt_imported INTERFACE IMPORTED)
    target_include_directories(nlopt_imported INTERFACE ${NLOPT_INCLUDE_DIRS})
    target_link_directories(nlopt_imported INTERFACE ${NLOPT_LIBRARY_DIRS})
    target_link_libraries(nlopt_imported INTERFACE ${NLOPT_LIBRARIES})
    set(NLOPT_LIB nlopt_imported)
endif()

# 4. Подключение libInterpolate без тестов Catch2
find_package(libInterpolate CONFIG QUIET)
if(NOT libInterpolate_FOUND)
    include(FetchContent)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(LIBINTERPOLATE_BUILD_TESTS OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        libInterpolate
        GIT_REPOSITORY https://github.com/CD3/libInterpolate.git
        GIT_TAG master
    )
    FetchContent_MakeAvailable(libInterpolate)
endif()

# Автоопределение экспортированной цели libInterpolate
if(TARGET libInterpolate::libInterpolate)
    set(LIBINTERPOLATE_LIB libInterpolate::libInterpolate)
elseif(TARGET libInterpolate)
    set(LIBINTERPOLATE_LIB libInterpolate)
elseif(TARGET Interpolate)
    set(LIBINTERPOLATE_LIB Interpolate)
else()
    add_library(libInterpolate_imported INTERFACE IMPORTED)
    target_include_directories(libInterpolate_imported INTERFACE
        ${libinterpolate_SOURCE_DIR}/src
        ${LIBINTERPOLATE_SOURCE_DIR}/src
    )
    set(LIBINTERPOLATE_LIB libInterpolate_imported)
endif()

# Исходные файлы библиотеки
set(TPRT_HEADERS
    include/tprt/Core/Types.hpp
    include/tprt/Core/ElasticTensor.hpp
    include/tprt/Geometry/Horizon.hpp
    include/tprt/Geometry/FlatHorizon.hpp
    include/tprt/Geometry/GridHorizon.hpp
    include/tprt/Model/Layer.hpp
    include/tprt/Model/VelocityModel.hpp
    include/tprt/Kinematics/Christoffel.hpp
    include/tprt/Kinematics/VTISlownessSpline.hpp
    include/tprt/Ray/RayCode.hpp
    include/tprt/Ray/RayTrajectory.hpp
    include/tprt/Ray/Ray.hpp
    include/tprt/Solver/TwoPointBendingTracer.hpp
    include/tprt/IO/JsonParser.hpp
)

set(TPRT_SOURCES
    src/Core/ElasticTensor.cpp
    src/Geometry/FlatHorizon.cpp
    src/Geometry/GridHorizon.cpp
    src/Model/Layer.cpp
    src/Model/VelocityModel.cpp
    src/Kinematics/Christoffel.cpp
    src/Kinematics/VTISlownessSpline.cpp
    src/Ray/Ray.cpp
    src/Solver/TwoPointBendingTracer.cpp
    src/IO/JsonParser.cpp
)

# Создание целевой библиотеки
add_library(tprt ${TPRT_SOURCES} ${TPRT_HEADERS})

target_include_directories(tprt
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(tprt
    PUBLIC
        Eigen3::Eigen
        ${NLOPT_LIB}
        ${LIBINTERPOLATE_LIB}
)

# Модуль юнит-тестирования
enable_testing()
find_package(GTest REQUIRED)

add_executable(tprt_autotests tests/autotests.cpp)
target_link_libraries(tprt_autotests PRIVATE tprt GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(tprt_autotests)