add_library(semaphores
            semaphores.c)
target_link_libraries(semaphores
                      ${catkin_LIBRARIES})

install(TARGETS semaphores
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(test)
