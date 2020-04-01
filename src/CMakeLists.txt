cmake_minimum_required(VERSION 3.8)

project(point_cloud_editor)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)

# Find VTK package
set(VTK_DIR /usr/local/lib/cmake/vtk-8.2)
find_package(VTK REQUIRED)
if(VTK_FOUND)
    MESSAGE("VTK library is found: ${VTK_INCLUDE_DIRS}")
else(VTK_FOUND)
    MESSAGE("VTK library not found.")
endif(VTK_FOUND)
include(${VTK_USE_FILE})

# Find PCL package
set(PCL_DIR /usr/share/pcl-1.9)
find_package(PCL 1.7.1 REQUIRED)
if(PCL_FOUND)
    MESSAGE("PCL library is found: ${PCL_INCLUDE_DIRS}")
else(PCL_FOUND)
    MESSAGE("PCL library not found.")
endif(PCL_FOUND)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp mainwindow.cpp)

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Qt5::Widgets)
