#pragma once
typedef unsigned short uint16_t;

void handle_message(
    uint16_t zero_message[],
    uint16_t message[],
    uint16_t sections_pwm[],
    uint16_t* section_channel,
    uint16_t non_hydraulic_actions[]
);