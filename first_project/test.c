#include <stdio.h>
#include <stdlib.h>
#include "core.h"

void DecToBin( int n ) {
    if ( n >= 2 ) {
        DecToBin( n/2 );
    }
    printf("%d", n % 2);
}
int main(int argc, char* argv[]) {
    int MESSAGE_SIZE = 6;
    if ((argc - 1) != (MESSAGE_SIZE)) {
        fprintf(stderr, "Error!\n");
        return EXIT_FAILURE;
    }
    int NUMBER_SECTIONS = 6;
    unsigned short message[MESSAGE_SIZE];
    unsigned short zero_message[MESSAGE_SIZE];
    for (int i = 1; i < argc; i++) {
        message[i - 1] = atoi(argv[i]);
        zero_message[i - 1] = 127;
    }
    unsigned short section_pwm[NUMBER_SECTIONS];
    int array_size = sizeof(section_pwm) / sizeof(section_pwm[0]);
    for (int i = 0; i < array_size; i++) {
        section_pwm[i] = 0;
    }
    unsigned short section_channel = 0;
    handle(zero_message, message, section_pwm, &section_channel);
    for (int i=0; i < array_size; i++) {
        printf("%i ", section_pwm[i]);
    }
    printf("\n");
    DecToBin(section_channel);
    // printf("%u\n", section_channel);
    printf("\n");
    return EXIT_SUCCESS;
}
