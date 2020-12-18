// #include "misc.h"

typedef unsigned short uint16_t;

void parse_section_pwm_and_channel_from_message(
    uint16_t zero_message[],
    uint16_t message[],
    uint16_t sections_pwm[],
    uint16_t* section_channel
);