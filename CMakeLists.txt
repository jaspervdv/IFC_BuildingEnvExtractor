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
find_package(tbb REQUIRED NO_DEFAULT_PATH)

# CJT
find_package(CJT REQUIRED)

# Boost
set(BCRYPT_LIBRARIES "bcrypt.lib") 
find_package(Boost REQUIRED)

# OpenCASCADE
set(OpenCASCADE_INCLUDE_DIR "C:/Users/Jasper/Documents/IfcOpenShell/IfcOpenshell/_deps-vs2019-x64-installed/opencascade-7.3.0p3/inc")
set(OpenCASCADEC_LIBRARY_DIR "C:/Users/Jasper/Documents/IfcOpenShell/IfcOpenshell/_deps-vs2019-x64-installed/opencascade-7.3.0p3/win64/lib")

# set location for IFCOpenShell
set(IFC_INCLUDE_DIR "C:/Users/Jasper/Documents/IfcOpenShell/IfcOpenshell/_installed-vs2019-x64/include")
set(IFC_LIBRARY_DIR "C:/Users/Jasper/Documents/IfcOpenShell/IfcOpenshell/_installed-vs2019-x64/lib")

include_directories("/usr/local/include" ${Boost_INCLUDE_DIRS} ${OpenCASCADE_INCLUDE_DIR} ${IFC_INCLUDE_DIR} ${CJT_INCLUDE_DIR}  )
link_directories("/usr/local/lib" ${Boost_LIBRARY_DIRS} ${OpenCASCADEC_LIBRARY_DIR} ${IFC_LIBRARY_DIR} ${CJT_LIBRARY_DIR})

set(IFC_LIBRARY_NAMES 
    IfcGeom IfcGeom_ifc2x3 IfcGeom_ifc4 IfcGeom_ifc4x1 IfcGeom_ifc4x2 IfcGeom_ifc4x3_rc1 IfcGeom_ifc4x3_rc2 
	IfcGeom_ifc4x3_rc3 IfcGeom_ifc4x3_rc4 IfcParse Serializers Serializers_ifc2x3 Serializers_ifc4 
	Serializers_ifc4x1 Serializers_ifc4x2 Serializers_ifc4x3_rc1 Serializers_ifc4x3_rc2 Serializers_ifc4x3_rc3 
	Serializers_ifc4x3_rc4
	)
string(REGEX REPLACE "([^;]+)" "${IFC_LIBRARY_DIR}/\\1.lib" IFC_LIBRARIES "${IFC_LIBRARY_NAMES}")

set(OCC_LIBRARY_NAMES
    TKernel TKMath TKBRep TKGeomBase TKGeomAlgo TKG3d TKG2d TKShHealing TKTopAlgo TKMesh TKPrim TKBool TKBO
    TKFillet TKSTEP TKSTEPBase TKSTEPAttr TKXSBase TKSTEP209 TKIGES TKOffset TKHLR
)
string(REGEX REPLACE "([^;]+)" "${OpenCASCADEC_LIBRARY_DIR}/\\1.lib" OCC_LIBRARIES "${OCC_LIBRARY_NAMES}")

add_executable(${PROJECT_NAME} 
main.cpp
inc/helper.cpp
inc/roomProcessor.cpp
)

foreach(LIB ${IFC_LIBRARIES})
    message(STATUS "IFC libraries are ${LIB}")
endforeach()

foreach(LIB ${OCC_LIBRARIES})
    message(STATUS "OCC libraries are ${LIB}")
endforeach()

foreach(LIB ${Boost_LIBRARIES})
    message(STATUS "Boost libraries are ${LIB}")
endforeach()


target_link_libraries(${PROJECT_NAME} ${OCC_LIBRARIES} ${IFC_LIBRARIES} ${Boost_LIBRARIES}  ${BCRYPT_LIBRARIES} ws2_32.lib CJT nlohmann_json::nlohmann_json)

# Adjust runtime environment
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY VS_DEBUGGER_ENVIRONMENT "PATH=$<$<CONFIG:DEBUG>:${OpenCASCADE_BINARY_DIR}d>$<$<NOT:$<CONFIG:DEBUG>>:${OpenCASCADE_BINARY_DIR}>;%PATH%;$<$<CONFIG:DEBUG>:${TBB_BINARY_DIR}d>$<$<NOT:$<CONFIG:DEBUG>>:${TBB_BINARY_DIR}>")