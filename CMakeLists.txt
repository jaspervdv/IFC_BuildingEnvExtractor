cmake_minimum_required(VERSION 3.10)

# set the project name
project(Ifc_Envelope_Extractor CXX)

FOREACH(flag CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL
		CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
		CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
	IF(${flag} MATCHES "/MD")
		STRING(REGEX REPLACE "/MD" "/MT" ${flag} "${${flag}}")
	ENDIF()
	IF(${flag} MATCHES "/MDd")
		STRING(REGEX REPLACE "/MDd" "/MTd" ${flag} "${${flag}}")
	ENDIF()
ENDFOREACH()

# nholmann
find_package(nlohmann_json REQUIRED)

# Tbb
find_package(TBB REQUIRED NO_DEFAULT_PATH)
SET(TBB_INCLUDE_DIR "tbb/include")
message(STATUS "${INTERFACE_INCLUDE_DIRECTORIES}")

# CJT
find_package(CJT REQUIRED)

# Boost
set(BCRYPT_LIBRARIES "bcrypt.lib") 
set(Boost_INCLUDE_DIR "C:/Program Files/boost/boost_1_77_0")
set(Boost_LIBRARY_DIR "C:/Program Files/boost/boost_1_77_0/lib64-msvc-14.2")

# OpenCASCADE
set(OpenCASCADE_INCLUDE_DIR "C:/Users/Jasper/Documents/IfcOpenShell/IfcOpenshell/_deps-vs2019-x64-installed/opencascade-7.3.0p3/inc")
set(OpenCASCADEC_LIBRARY_DIR "C:/Users/Jasper/Documents/IfcOpenShell/IfcOpenshell/_deps-vs2019-x64-installed/opencascade-7.3.0p3/win64/lib")


# set location for IFCOpenShell
set(IFC_INCLUDE_DIR "C:/Users/Jasper/Documents/IfcOpenShell/IfcOpenshell/_installed-vs2019-x64/include")
set(IFC_LIBRARY_DIR "C:/Users/Jasper/Documents/IfcOpenShell/IfcOpenshell/_installed-vs2019-x64/lib")

include_directories("/usr/local/include" ${Boost_INCLUDE_DIR} ${OpenCASCADE_INCLUDE_DIR} ${IFC_INCLUDE_DIR} ${CJT_INCLUDE_DIR}  )
link_directories("/usr/local/lib" ${Boost_LIBRARY_DIR} ${OpenCASCADEC_LIBRARY_DIR} ${IFC_LIBRARY_DIR} ${CJT_LIBRARY_DIR})

file(GLOB IFC_LIBRARY_FILES "${IFC_LIBRARY_DIR}/*.lib")
file(GLOB OCC_LIBRARY_FILES "${OpenCASCADEC_LIBRARY_DIR}/*.lib")

add_executable(${PROJECT_NAME} 
main.cpp
inc/cjCreator.cpp
inc/dataManager.cpp
inc/helper.cpp
inc/ioManager.cpp
inc/settingsCollection.cpp
inc/surfaceCollection.cpp
inc/voxel.cpp
inc/voxelGrid.cpp
)

foreach(LIB ${IFC_LIBRARY_FILES})
    message(STATUS "IFC libraries are ${LIB}")
endforeach()

foreach(LIB ${OCC_LIBRARY_FILES})
    message(STATUS "OCC libraries are ${LIB}")
endforeach()

foreach(LIB ${Boost_LIBRARIES})
    message(STATUS "Boost libraries are ${LIB}")
endforeach()

target_link_libraries(${PROJECT_NAME} ${OCC_LIBRARY_FILES} ${IFC_LIBRARY_FILES} ${Boost_LIBRARIES} ${BCRYPT_LIBRARIES} ws2_32.lib TBB::tbb CJT nlohmann_json::nlohmann_json)

# Adjust runtime environment
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY VS_DEBUGGER_ENVIRONMENT "PATH=$<$<CONFIG:DEBUG>:${OpenCASCADE_BINARY_DIR}d>$<$<NOT:$<CONFIG:DEBUG>>:${OpenCASCADE_BINARY_DIR}>;%PATH%;$<$<CONFIG:DEBUG>:${TBB_BINARY_DIR}d>$<$<NOT:$<CONFIG:DEBUG>>:${TBB_BINARY_DIR}>")