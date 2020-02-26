cmake_minimum_required(VERSION 2.6)
project(cpp_examples)

add_executable(cpp_examples main.cpp)

add_executable(factorial_example factorial_example.cpp 
                                        Factorial.cpp)
                                        
install(TARGETS cpp_examples RUNTIME DESTINATION bin)
