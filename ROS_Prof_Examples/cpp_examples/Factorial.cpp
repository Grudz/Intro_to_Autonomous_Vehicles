
#include "Factorial.h"

using namespace cpp_example;

Factorial::Factorial(){
    n_ = 3;
}    

Factorial::Factorial(int n){
    n_ = n;
}
    
int Factorial::compute(){
    int result = 1;
    
    for(int i=0; i<n_; i++){
        result = result * (n_- i);
    }
    return result;
}
    
