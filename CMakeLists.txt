cmake_minimum_required(VERSION 3.10)

project(parameterize VERSION 1.0)
include(FetchContent)

set(LXSDK_PATH "../LXSDK-000025")
set(CGAL_DIR "../CGAL-6.0.1")
set(LIBIGL_PATH "../libigl-2.5.0")
set(EIGEN_PATH "../eigen-3.4.0")

# Set your Boost path
set(Boost_INCLUDE_DIR "../boost_1_87_0")

# Set your GMP and MPFR libraries path
set(LOCAL_LIB_DIR /usr/local/opt)
set(GMP_INCLUDE_DIR "${LOCAL_LIB_DIR}/gmp/include")
set(MPFR_INCLUDE_DIR "${LOCAL_LIB_DIR}/mpfr/include")
set(GMP_LIBRARIES ${LOCAL_LIB_DIR}/gmp/lib/libgmp.a ${LOCAL_LIB_DIR}/gmp/lib/libgmpxx.a)
set(MPFR_LIBRARIES ${LOCAL_LIB_DIR}/mpfr/lib/libmpfr.a)

if (NOT LXSDK_PATH)
    FetchContent_Declare( 
        lxsdk
        URL https://thefoundry.s3.us-east-1.amazonaws.com/products/modo/16.1v5/lxsdk16.1v5_692025.zip
    )
    FetchContent_Populate(lxsdk)
    message(STATUS "FetchContent lxsdk directory: ${lxsdk_SOURCE_DIR}")
    set(LXSDK_PATH ${lxsdk_SOURCE_DIR})
endif()

# Get all source and headers for lxsdk
file(GLOB LXSDK_SOURCES ${LXSDK_PATH}/common/*.cpp)
file(GLOB LXSDK_HEADERS ${LXSDK_PATH}/include/*.h?)

add_library(lxsdk
    STATIC
        ${LXSDK_SOURCES}
)

set_target_properties(lxsdk 
    PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY lib
        POSITION_INDEPENDENT_CODE ON
)

target_compile_definitions(lxsdk
    PRIVATE
        GL_SILENCE_DEPRECATION
)

target_compile_features(lxsdk
    PRIVATE
        cxx_std_17
)

target_include_directories(lxsdk
    PRIVATE
        ${LXSDK_PATH}/include
)

if (NOT CGAL_DIR)
    FetchContent_Declare( 
        cgal
        GIT_REPOSITORY https://github.com/CGAL/cgal.git
        GIT_TAG        v6.0.1
    )
    FetchContent_Populate(cgal)
    message(STATUS "FetchContent cgal directory: ${cgal_SOURCE_DIR}")
    set(CGAL_DIR ${cgal_SOURCE_DIR})
endif()

if (NOT EIGEN_PATH)
    FetchContent_Declare( 
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
    )
    FetchContent_Populate(eigen)
    message(STATUS "FetchContent eigen directory: ${eigen_SOURCE_DIR}")
    set(EIGEN_PATH ${eigen_SOURCE_DIR})
endif()

if (NOT LIBIGL_PATH)
    FetchContent_Declare( 
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG        v2.5.0
    )
    FetchContent_Populate(libigl)
    message(STATUS "FetchContent libigl directory: ${libigl_SOURCE_DIR}")
    set(LIBIGL_PATH ${libigl_SOURCE_DIR})
endif()

find_package(CGAL REQUIRED) 
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)

message(STATUS "GMP_LIBRARIES: ${GMP_LIBRARIES}")
message(STATUS "MPFR_LIBRARIES: ${MPFR_LIBRARIES}")

add_library(parameterize
    SHARED
        "source/tool.cpp"
        "source/parameterize.cpp"
)

target_compile_features(parameterize
    PRIVATE
        cxx_std_17
)


# Set your CGAL include directry
target_include_directories(parameterize 
    PUBLIC
        ${LXSDK_PATH}/include
        ${Boost_INCLUDE_DIR}
        ${CGAL_INCLUDE_DIRS}
    	${LIBIGL_PATH}/include
    	${EIGEN_PATH}
)

#set_target_properties(triangulate PROPERTIES LINK_SEARCH_START_STATIC TRUE)

# Set your CGAL libraray 
target_link_libraries(parameterize
    PUBLIC
	    lxsdk
        ${GMP_LIBRARIES}
        ${MPFR_LIBRARIES}
        CGAL::CGAL
)

# Install plugins and configs into kit directry
install(
    FILES
        "index.cfg"
        "index.html"
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
)
install(
    DIRECTORY
        "images"
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
)
install(
    TARGETS
        parameterize
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}/extra
)
