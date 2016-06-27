cmake_minimum_required (VERSION 2.6 FATAL_ERROR) 

project      (realsense_grabber) 
find_package (Qt4 REQUIRED) 
find_package (PCL 1.7.1 REQUIRED)
find_package (VTK REQUIRED) 
#include(${PCL_USE_FILE})


include(CheckCXXCompilerFlag) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
include_directories (/home/sweet/git-project/pcl/io/include/pcl)
include_directories (/usr/local/include/pcl-1.8)
include_directories (/home/sweet/git-project/librealsense/include/)
link_directories    (${PCL_LIBRARY_DIRS} /home/sweet/git-project/pcl/pcl-build) 
include_directories (${VTK_INCLUDE_DIRS} /usr/include/vtk-5.8)
#include_directories (${PCL_INCLUDE_DIRS} /home/sweet/git-project/librealsense/include/librealsense) 
add_definitions     (${PCL_DEFINITIONS})


set  (project_SOURCES real_sense_viewer.cpp real_sense_common.cpp real_sense_grabber.cpp real_sense_device_manager.cpp)  
set  (project_HEADERS real_sense_common.h real_sense_grabber.h real_sense_device_manager.h) 
set  (project_FORMS   ) 
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid  QVTK) 


QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS}) 
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS}) 

INCLUDE         (${QT_USE_FILE}) 
ADD_DEFINITIONS (${QT_DEFINITIONS}) 

ADD_EXECUTABLE  (realsense_grabber ${project_SOURCES} 
                                ${project_FORMS_HEADERS} 
                                ${project_HEADERS_MOC}) 

#TARGET_LINK_LIBRARIES (realsense_grabber ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} /usr/local/lib/librealsense.so ) 
TARGET_LINK_LIBRARIES (realsense_grabber ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} /usr/local/lib/librealsense.so)
include(${VTK_USE_FILE}) 