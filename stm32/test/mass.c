#include <stdio.h>


void foo(int *array_ptr, int array_size) {
    for (int i=0;i<array_size;i++) {
        printf("%i\n", array_ptr[i]);
    }
    
}

int main() {
    int array[] = {1, 2, 3, 4};
    int array_size = sizeof(array) / sizeof(array[0]);
    printf("%i\n", array_size);
    foo(array, array_size);
}