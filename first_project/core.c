#pragma once
#include "core.h"

uint16_t mode1[] = {0, 1, 2, 3}; // режимы задают какие секции управляются какими ждойстиками 
uint16_t mode2[] = {4, 5, 65535, 65535}; // 65535 -- заглушка, недопустимое значение номера секции, есть проверка
uint16_t PWM_THRESHOLD = 30; // значение порога шим, ниже которого 0

void get_potenciometer_values_from_message(uint16_t message[],
                                           uint16_t potenciometers_values[4]) {
    for (int i = 0; i < 4; i++) {
        potenciometers_values[i] = message[i];
    }
}

void get_button_values_from_message(uint16_t message[],
                                    uint16_t buttons_values[4]) {
    buttons_values[0] = message[4];
    buttons_values[1] = message[5];
}

void potentiometer_to_pwm(uint16_t potenciometers_values[4],
                          uint16_t zero_potenciometers_values[4],
                          uint16_t pwm_values[4]) {
    for (int i=0; i < 4; i++) {
        int absolute_value = abs(potenciometers_values[i] - zero_potenciometers_values[i]);

        if (absolute_value >= PWM_THRESHOLD) {
            pwm_values[i] = absolute_value;
        } else {
            pwm_values[i] = 0;
        }
    }
}

void potentiometer_to_channels(uint16_t potenciometers_values[4],
                               uint16_t zero_potenciometers_values[4],
                               uint16_t sections_channels[4]) {
    for (int i=0; i < 4; i++) {
        if (potenciometers_values[i] >= zero_potenciometers_values[i]) {
            sections_channels[i] = 0;
        } else {
            sections_channels[i] = 1;
        }
    }
}

void get_mode_by_button_value(uint16_t buttons_values[2],
                              uint16_t mode[]) {
    if (buttons_values[0] == 0) {
        for (int i=0; i<4; i++) {
            mode[i] = mode1[i];
        }
        
    } else {
        for (int i=0; i<4; i++) {
            mode[i] = mode2[i];
        }
    }
}

int check_section_exist(int section_index) {
    if (section_index < 6) {
        return 1;
    } else {
        return 0;
    }
}

void handle(
    uint16_t zero_message[],
    uint16_t message[],
    uint16_t sections_pwm[],
    uint16_t* section_channel
) {
    uint16_t potenciometers_values[4];
    get_potenciometer_values_from_message(message, potenciometers_values);
    uint16_t zero_potenciometers_values[4];
    get_potenciometer_values_from_message(zero_message, zero_potenciometers_values);
    uint16_t buttons_values[2];
    get_button_values_from_message(message, buttons_values);
    uint16_t pwm_values[4];
    potentiometer_to_pwm(potenciometers_values, zero_potenciometers_values, pwm_values);
    uint16_t sections_channels[4];
    potentiometer_to_channels(potenciometers_values, zero_potenciometers_values, sections_channels);
    uint16_t mode[4];
    get_mode_by_button_value(buttons_values, mode);

    for (int i=0; i < 4; i++) {
        int section_index = mode[i];

        if (!check_section_exist(section_index)) {
            continue;
        }

        int channel_counter = (section_index * 2);

        if (pwm_values[i] > 0) {
            sections_pwm[section_index] = pwm_values[i];
            // set channel
            *section_channel |= ((1 << channel_counter) << sections_channels[i]);
        } else {
            // reset channel
            *section_channel &= ~((1 << channel_counter) << sections_channels[i]);
        }
        // reset neighboring channel
        *section_channel &= ~((1 << channel_counter) << (1 - sections_channels[i]));
    }
}
