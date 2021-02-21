add_executable(test_manual_rs232.exe
               test_manual_rs232.cpp)
target_link_libraries(test_manual_rs232.exe
                      rs232)

add_executable(test_manual_screen.exe
               test_manual_screen.cpp)
target_link_libraries(test_manual_screen.exe
                      rs232)
