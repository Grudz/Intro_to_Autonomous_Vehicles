#include "Factorial.h"
#include <stdio.h>

int main(int argc, char** argv){
    
    cpp_example::Factorial f;
    cpp_example::Factorial g(5);
    
    
    
    int result_1 = f.compute();
    int result_2 = g.compute();
    
    printf("Result 1: %d\n", result_1);
    
    printf("Result 2: %d\n", result_2);
    
    return 0; 
}
