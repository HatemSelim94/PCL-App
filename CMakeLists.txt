cmake_minimum_required(VERSION 3.5)

project(PclApp LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()
find_package(PCL 1.8.1 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Boost)

FIND_PACKAGE(VTK)
INCLUDE(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


if(ANDROID)
  add_library(PclApp SHARED
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    pclprocessor.h
    pclprocessor.cpp
  )
else()
  add_executable(PclApp
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    pclprocessor.h
    pclprocessor.cpp
  )
endif()

target_link_libraries(PclApp PRIVATE Qt5::Widgets ${PCL_LIBRARIES} ${Boost_LIBRARIES} vtkGUISupportQt)
