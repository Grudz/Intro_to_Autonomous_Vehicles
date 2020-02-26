#include <stdio.h>
#include <cstdlib>

int main(int argc, char** argv){
    
    printf("Argc value: %d", argc);
    if(argc != 3){
        printf("Incorrect number of arguements provided to this program. Three are required \n");
    } else {
        printf("Arguement List:\n");
        for(int i=0; i<argc; i++){
                printf("%d: %s\n", i, argv[i]);
        }

        int a = std::atoi(argv[1]);
        int b = std::atoi(argv[2]);
        int result = a + b;
        
        printf("%d + %d = %d\n", a, b, result); 
    }
    
    
    
    return 0;
}
