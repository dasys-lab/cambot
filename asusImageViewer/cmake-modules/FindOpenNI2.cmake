find_path(OPENNI2_INCLUDE_DIRS "OpenNI.h"
    PATHS
        ${CMAKE_SOURCE_DIR} /usr/include/openni2
    DOC "OpenNI c++ interface header")
find_library(OPENNI2_LIBRARY "OpenNI2"
    PATHS
        /usr/include/openni2
    DOC "OpenNI2 library")

if(OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARY)
    set(OPENNI2_FOUND TRUE)
endif()

#--- Notifications
if(OPENNI2_FOUND AND NOT OpenNI2_FIND_QUIETLY) 
    message(STATUS "Found OpenNI: ${OPENNI2_LIBRARY}")
else()
    if(OPENNI2_FIND_REQUIRED)
        message(STATUS OPENNI2_INCLUDE_DIRS: ${OPENNI2_INCLUDE_DIRS})
        message(STATUS OPENNI2_LIBRARY:     ${OPENNI2_LIBRARY})
        message(FATAL_ERROR "COULD not find OPENNI2")
    endif()
endif()
