cmake_minimum_required(VERSION 3.11)
project(fast_classes)

set(FAST_CLASSES_SOURCE_FILES
    src/featurecnm.cpp
    )


add_library(fast_classes
    ${FAST_CLASSES_SOURCE_FILES}
    )

find_package(OpenCV REQUIRED)

target_link_libraries(fast_classes PUBLIC
                ${OpenCV_LIBS}
                )

target_include_directories(fast_classes PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
    )