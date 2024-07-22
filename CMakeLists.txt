cmake_minimum_required(VERSION 3.10)

# set the project name
project(Ifc_Envelope_Extractor CXX)
set(CMAKE_CXX_STANDARD 17)

option(USE_MMAP "Adds a command line options to parse IFC files from memory mapped files using Boost.Iostreams" OFF)

# nholmann
find_package(nlohmann_json REQUIRED)

# IFCOpenShell
if("${IFC_INCLUDE_DIR}" STREQUAL "")
    if(UNIX)
        find_path(IFC_INCLUDE_DIR ifcgeom/IfcGeom.h
        PATHS
        /usr/local/include/ifcgeom/
        )
    endif()

    if(NOT IFC_INCLUDE_DIR)
        message(FATAL_ERROR "Unable to find IFCOpenShell include directory. Specify IFC_INCLUDE_DIR manually")
    endif()
endif()

if(IFC_INCLUDE_DIR)
    set(IFC_INCLUDE_DIR ${IFC_INCLUDE_DIR} CACHE FILEPATH "IFCOpenShell header files")
    message(STATUS "Looking for IFCOpenShell include files in: ${IFC_INCLUDE_DIR}")

    if(UNIX)
        set(IFCGEOM_INCLUDE_DIR "${IFC_INCLUDE_DIR}/ifcgeom")
        set(IFCGEOMSCHEMA_INCLUDE_DIR "${IFC_INCLUDE_DIR}/ifcgeom_schema_agnostic")
        set(IFCPARSE_INCLUDE_DIR "${IFC_INCLUDE_DIR}/ifcparse")
        message(STATUS "Looking for ifcgeom include files in: ${IFCGEOM_INCLUDE_DIR}")
        message(STATUS "Looking for ifcgeom_schema_agnostic include files in: ${IFCGEOMSCHEMA_INCLUDE_DIR}")
        message(STATUS "Looking for ifcparse include files in: ${IFCPARSE_INCLUDE_DIR}")
    endif(UNIX)
endif()

set(IFC_LIBRARY_NAMES 
Serializers    
Serializers_ifc2x3 Serializers_ifc4 Serializers_ifc4x1 Serializers_ifc4x2 Serializers_ifc4x3 Serializers_ifc4x3_add1 Serializers_ifc4x3_add2
Serializers_ifc4x3_rc1 Serializers_ifc4x3_rc2 Serializers_ifc4x3_rc3 Serializers_ifc4x3_rc4 Serializers_ifc4x3_tc1 
IfcGeom
IfcGeom_ifc2x3 IfcGeom_ifc4 IfcGeom_ifc4x1 IfcGeom_ifc4x2 IfcGeom_ifc4x3 IfcGeom_ifc4x3_add1 IfcGeom_ifc4x3_add2 
IfcGeom_ifc4x3_rc1 IfcGeom_ifc4x3_rc2 IfcGeom_ifc4x3_rc3 IfcGeom_ifc4x3_rc4 IfcGeom_ifc4x3_tc1
Serializers IfcParse 
)

if("${IFC_LIBRARY_DIR}" STREQUAL "")
    if(UNIX)
        find_path(IFC_LIBRARY_DIR libIfcGeom.a
        PATHS
        ${IFC_INCLUDE_DIR}/../lib/
    )
    elseif(WIN32)
        find_path(IFC_LIBRARY_DIR IfcGeom.lib
        PATHS
        ${IFC_INCLUDE_DIR}/../lib/
    )
    endif()

    if(IFC_LIBRARY_DIR)
        message(STATUS "Found IFCOpenShell lib files in: ${IFC_LIBRARY_DIR}")
    else()
        message(FATAL_ERROR "Unable to find IFCOpenShell lib directory. Specify IFC_LIBRARY_DIR manually")    
    endif()
else()
    set(IFC_LIBRARY_DIR ${IFC_LIBRARY_DIR} CACHE FILEPATH "IFCOpenShell header files")
    message(STATUS "Looking for IFCOpenShell lib files in: ${IFC_LIBRARY_DIR}")
endif()

if(WIN32)
    file(GLOB IFC_LIBRARY_FILES "${IFC_LIBRARY_DIR}/*.lib")
elseif(UNIX)
    foreach(file ${IFC_LIBRARY_NAMES})
        list(APPEND IFC_LIBRARY_FILES "${IFC_LIBRARY_DIR}/lib${file}.a")
    endforeach()
endif()

# OpenCascade
if("${OCC_INCLUDE_DIR}" STREQUAL "")
    find_path(OCC_INCLUDE_DIR Standard_Version.hxx
    PATHS
        /usr/include/occt
        /usr/include/oce
        /usr/include/opencascade
    )

    if(OCC_INCLUDE_DIR)
        message(STATUS "Found Open CASCADE include files in: ${OCC_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Unable to find Open CASCADE include directory. Specify OCC_INCLUDE_DIR manually")
    endif()
else()
    set(OCC_INCLUDE_DIR ${OCC_INCLUDE_DIR} CACHE FILEPATH "Open CASCADE header files")
    message(STATUS "Looking for Open CASCADE include files in: ${OCC_INCLUDE_DIR}")
endif()

# get OCCT version
if(OCC_INCLUDE_DIR)
    file(STRINGS ${OCC_INCLUDE_DIR}/Standard_Version.hxx OCC_VERSION
        REGEX "#define OCC_VERSION_COMPLETE*"
    )
    string(REGEX MATCH "[0-9.]+" OCC_VERSION_STRING ${OCC_VERSION})
    
    message(STATUS "Open CASCADE Version: " ${OCC_VERSION_STRING})
endif(OCC_INCLUDE_DIR)

set(OPENCASCADE_LIBRARY_NAMES
TKernel TKMath TKBRep TKGeomBase TKGeomAlgo TKG3d TKG2d TKShHealing TKTopAlgo TKMesh TKPrim TKBool TKBO
TKFillet TKXSBase TKOffset TKHLR TKBin TKV3d
)

if(OCC_VERSION_STRING VERSION_LESS 7.8.0)
    list(APPEND OPENCASCADE_LIBRARY_NAMES  TKIGES TKSTEPBase TKSTEPAttr TKSTEP209 TKSTEP)
else(OCC_VERSION_STRING VERSION_LESS 7.8.0)
    list(APPEND OPENCASCADE_LIBRARY_NAMES TKDESTEP TKDEIGES)
endif(OCC_VERSION_STRING VERSION_LESS 7.8.0)

if("${OCC_LIBRARY_DIR}" STREQUAL "")
    if(WIN32)
        find_path(OCC_LIBRARY_DIR TKernel.lib
            PATHS
            ${OCC_INCLUDE_DIR}/../win64/lib #TODO: add more paths
            ${OCC_INCLUDE_DIR}/../win64/vc14/lib
    )
    else()
        find_library(OCC_LIBRARY TKernel
            PATHS
            /usr/lib
    )
    endif()

    if(OCC_LIBRARY_DIR)
        message(STATUS "Found Open CASCADE library files in: ${OCC_LIBRARY_DIR}")
    elseif(OCC_LIBRARY)
        GET_FILENAME_COMPONENT(OCC_LIBRARY_DIR ${OCC_LIBRARY} PATH)
        message(STATUS "Found Open CASCADE library files in: ${OCC_LIBRARY_DIR}")
    else()
        message(FATAL_ERROR "Unable find Open CASCADE library directory, specify OCC_LIBRARY_DIR manually.")
    endif()
else()
    set(OCC_LIBRARY_DIR ${OCC_LIBRARY_DIR} CACHE FILEPATH "Open CASCADE library files")
    message(STATUS "Looking for Open CASCADE library files in: ${OCC_LIBRARY_DIR}")
endif()

mark_as_advanced(libTKernel)
find_library(libTKernel NAMES TKernel TKerneld PATHS ${OCC_LIBRARY_DIR})

if(libTKernel)
    message(STATUS "Required Open Cascade Library files found")
else()
    message(FATAL_ERROR "Unable to find Open Cascade library files, aborting")
endif()

# Use the found libTKernel as a template for all other OCC libraries
foreach(lib ${OPENCASCADE_LIBRARY_NAMES})
    string(REPLACE TKerneld "${lib}" lib_path "${libTKernel}")
    string(REPLACE TKernel "${lib}" lib_path "${lib_path}")
    list(APPEND OPENCASCADE_LIBRARIES "${lib_path}")
endforeach()

if(WIN32)
    list(APPEND OPENCASCADE_LIBRARIES ws2_32.lib)
endif()

get_filename_component(libTKernelExt ${libTKernel} EXT)
if("${libTKernelExt}" STREQUAL ".a")
    set(OCCT_STATIC ON)
endif()

if(OCCT_STATIC)
    find_package(Threads)
    if (NOT WIN32)
        set(OPENCASCADE_LIBRARIES ${OPENCASCADE_LIBRARIES} "dl")
    endif()
endif()

# Boost
if(WIN32 AND("$ENV{CONDA_BUILD}" STREQUAL ""))
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC_RUNTIME OFF)
    set(Boost_USE_MULTITHREADED ON)

    # Disable Boost's autolinking as the libraries to be linked to are supplied
    # already by CMake, and wrong libraries would be asked for when code is
    # compiled with a toolset different from default.
    if(MSVC)
        add_definitions(-DBOOST_ALL_NO_LIB)

        # Necessary for boost version >= 1.67
        set(BCRYPT_LIBRARIES "bcrypt.lib")
    endif()
else()
    # Disable Boost's autolinking as the libraries to be linked to are supplied
    # already by CMake, and it's going to conflict if there are multiple, as is
    # the case in conda-forge's libboost feedstock.
    add_definitions(-DBOOST_ALL_NO_LIB)

    if(WIN32)
        # Necessary for boost version >= 1.67
        set(BCRYPT_LIBRARIES "bcrypt.lib")
    endif()
endif()

set(BOOST_COMPONENTS ${BOOST_COMPONENTS} filesystem)
if(USE_MMAP)
    set(BOOST_COMPONENTS ${BOOST_COMPONENTS} iostreams)
    add_definitions(-DUSE_MMAP)
endif()

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})

message(STATUS "Boost include files found in ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost libraries found in ${Boost_LIBRARIES}")

# CJT
if("${CJT_INCLUDE_DIR}" STREQUAL "")
    find_package(CJT REQUIRED)

    if(CJT_DIR)
        message(STATUS "Found CJT in: ${CJT_DIR}")
    else()
        message(ERROR "Unable to find CJT. Specify CJT location manually")
    endif()
else()
    set(CJT_DIR ${CJT_DIR} CACHE FILEPATH "CJT cmake dir")
    message(STATUS "Looking for CJT include files in: ${CJT_INCLUDE_DIR}")
    message(STATUS "Looking for CJT lib files in: ${CJT_LIBRARY_DIR}")
endif()

if(UNIX)
    include_directories("/usr/local/include" ${Boost_INCLUDE_DIRS} ${OCC_INCLUDE_DIR} ${IFC_INCLUDE_DIR} ${IFCGEOM_INCLUDE_DIR} ${IFCGEOMSCHEMA_INCLUDE_DIR} ${IFCPARSE_INCLUDE_DIR} ${CJT_INCLUDE_DIR}  )
elseif(WIN32)
    include_directories("/usr/local/include" ${Boost_INCLUDE_DIRS} ${OCC_INCLUDE_DIR} ${IFC_INCLUDE_DIR} ${CJT_INCLUDE_DIR}  )
endif()

add_executable(${PROJECT_NAME} 
main.cpp
inc/cjCreator.cpp
inc/DataManager.cpp
inc/helper.cpp
inc/IOManager.cpp
inc/settingsCollection.cpp
inc/stringManager.cpp
inc/surfaceCollection.cpp
inc/voxel.cpp
inc/voxelGrid.cpp
)

link_directories("/usr/local/lib" ${Boost_LIBRARY_DIRS} ${OCC_LIBRARY_DIR} ${IFC_LIBRARY_DIR} ${CJT_LIBRARY_DIR})

if(UNIX)
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${BCRYPT_LIBRARIES} ${CJT_Foundation_LIBRARIES} nlohmann_json::nlohmann_json)
elseif(WIN32)
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${BCRYPT_LIBRARIES} ws2_32.lib ${CJT_Foundation_LIBRARIES} nlohmann_json::nlohmann_json)
endif()

message(STATUS "Boost libraries are:")
foreach(LIB ${Boost_LIBRARIES})
    message(STATUS "    ${LIB}")
endforeach()

message(STATUS "IFC libraries are:")
foreach(LIB ${IFC_LIBRARY_FILES})
    message(STATUS "    ${LIB}")
    target_link_libraries(${PROJECT_NAME} ${LIB})
endforeach()

message(STATUS "OCC libraries are:")
foreach(LIB ${OPENCASCADE_LIBRARIES})
    message(STATUS "    ${LIB}")
    target_link_libraries(${PROJECT_NAME} ${LIB})
endforeach()

get_target_property(LINK_LIBS ${PROJECT_NAME} LINK_LIBRARIES)
message(STATUS "Linked libraries for ${PROJECT_NAME}: ${LINK_LIBS}")