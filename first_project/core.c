#pragma once
#include "core.h"

// available modes
uint16_t available_modes[2] = {1, 2};
uint16_t PWM_THRESHOLD = 30; // значение порога шим, ниже которого 0

uint16_t working_sections_table[5][4] = { // i - mode, j - section index
    65535, 65535, 65535, 65535,
    0, 1, 2, 3,
    0, 1, 4, 5,
    6, 7, 8, 9,
    6, 7, 10, 11
};

int is_mode_available(uint16_t val, uint16_t arr[], int number_of_elements){
    for(int i = 0; i < number_of_elements; i++){
        if(arr[i] == val) {
            return 1;
        }
    }
    return 0;
}

void get_potenciometer_values_from_message(uint16_t message[],
                                           uint16_t potenciometers_values[4]) {
    for (int i = 0; i < 4; i++) {
        potenciometers_values[i] = message[i];
    }
}

void get_mode_from_message(uint16_t message[], uint16_t *mode) {
    *mode = message[4];
}

uint16_t filter_pwm_by_threshold(uint16_t pwm_value, uint16_t PWM_THRESHOLD) {
    if (pwm_value >= PWM_THRESHOLD) {
        return pwm_value;
    } else {
        return 0;
    }
}

void get_pwm_from_potenciometer(uint16_t potenciometers_values[4],
                          uint16_t zero_potenciometers_values[4],
                          uint16_t pwm_values[4]) {
    for (int i=0; i < 4; i++) {
        int absolute_value = abs(potenciometers_values[i] - zero_potenciometers_values[i]);
        
        pwm_values[i] = filter_pwm_by_threshold(absolute_value, PWM_THRESHOLD);
    }
}

void get_section_channel_from_potenciometer(uint16_t potenciometers_values[4],
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

int check_section_exist(int section_index) {
    if (section_index < 11) {
        return 1;
    } else {
        return 0;
    }
}

int check_power(uint16_t buttons_values[3]) {
  int POWER = 0;
  if (buttons_values[0]) {
    POWER = 1;
    return POWER;
  } else {
    return POWER;
  }
}

void set_pwm_to_sections(uint16_t sections_pwm[],
                         uint16_t pwm_values[4],
                         uint16_t working_sections[4]) {
    for (int i=0; i < 4; i++) {
        int section_index = working_sections[i];

        if (!check_section_exist(section_index)) {
            continue;
        }

        int channel_counter = (section_index * 2);

        if (pwm_values[i] > 0) {
            sections_pwm[section_index] = pwm_values[i];
        }
    }
}


void set_channels_to_sections(
    uint16_t* section_channel,
    uint16_t sections_channels[4],
    uint16_t pwm_values[4],
    uint16_t working_sections[4]) {
    for (int i=0; i < 4; i++) {
        int section_index = working_sections[i];

        if (!check_section_exist(section_index)) {
            continue;
        }

        int channel_counter = (section_index * 2);

        if (pwm_values[i] > 0) {
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

void handle_message(
    uint16_t zero_message[],
    uint16_t message[],
    uint16_t sections_pwm[],
    uint16_t* section_channel
) {
    int mode = 0;
    get_mode_from_message(message, &mode);
    if (!is_mode_available(mode, available_modes, 2)) {
        return;
    }
    uint16_t potenciometers_values[4];
    get_potenciometer_values_from_message(message, potenciometers_values);
    uint16_t zero_potenciometers_values[4];
    get_potenciometer_values_from_message(zero_message, zero_potenciometers_values);
    uint16_t pwm_values[4];
    get_pwm_from_potenciometer(potenciometers_values, zero_potenciometers_values, pwm_values);
    uint16_t sections_channels[4];
    get_section_channel_from_potenciometer(potenciometers_values, zero_potenciometers_values, sections_channels);

    set_pwm_to_sections(sections_pwm, pwm_values, working_sections_table[mode]);
    set_channels_to_sections(section_channel, sections_channels, pwm_values, working_sections_table[mode]);
}
