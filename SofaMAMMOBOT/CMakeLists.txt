cmake_minimum_required(VERSION 3.12)
project(SofaMAMMOBOT VERSION 1.0)

include(cmake/environment.cmake)

#SOFA packages containing our dependencies
find_package(SofaFramework REQUIRED)
sofa_find_package(SofaPython3 QUIET)

set(SofaPythonAny_FOUND FALSE)

if(SofaPython3_FOUND)
	message("--Found SofaPython3")
	SP3_add_python_package(
		SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python3/src/SofaMAMMOBOT
		TARGET_DIRECTORY SofaMAMMOBOT
	)
	set(SofaPythonAny_FOUND TRUE)
endif()

set(HEADER_FILES src/SofaMAMMOBOT/config.h)
set(SOURCE_FILES src/SofaMAMMOBOT/initSofaMAMMOBOT.cpp)

file(GLOB_RECURSE RESOURCE_FILES "*.md" "*.psl" "*.py" "*.pyscn" "*.scn" "*.rst")

#SOFA plugin are basically libraries
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

#Dependencies
target_link_libraries(${PROJECT_NAME} SofaCore)

#nested-namespace-definition requires compiler flag '/std:c++17'
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SOFAEversion")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

#Create package Congif, Version & Target files.
#Deploy the headers, resources, scenes & examples
sofa_create_package_with_targets(
	PACKAGE_NAME ${PROJECT_NAME}
	PACKAGE_VERSION ${PROJECT_VERSION}
	TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
	INCLUDE_SOURCE_DIR "src"
	INCLUDE_INSTALL_DIR ${PROJECT_NAME}
	RELOCATABLE "plugins"
	)