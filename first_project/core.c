#pragma once
#include "core.h"

uint16_t mode1[] = {0, 1, 2, 3}; // режимы задают какие секции управляются какими ждойстиками 
uint16_t mode2[] = {4, 5, 65535, 65535}; // 65535 -- заглушка, недопустимое значение номера секции, есть проверка
uint16_t PWM_THRESHOLD = 30; // значение порога шим, ниже которого 0

uint16_t HYDRO_DOWN = 1; // TRUE / FALSE if using this board
uint16_t HYDRO_UP = 0; // TRUE / FALSE if using this board

void get_potenciometer_values_from_message(uint16_t message[],
                                           uint16_t potenciometers_values[4]) {
    for (int i = 0; i < 4; i++) {
        potenciometers_values[i] = message[i];
    }
}

void get_button_values_from_message(uint16_t message[],
                                    uint16_t buttons_values[3]) {
    buttons_values[0] = message[4];
    buttons_values[1] = message[5];
    buttons_values[2] = message[6];
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

void get_working_sections_by_button_value(uint16_t buttons_values[3],
                                  uint16_t sections[]) {
    if (buttons_values[2] == 0) {
        for (int i=0; i<4; i++) {
            sections[i] = mode1[i];
        }
        
    } else {
        for (int i=0; i<4; i++) {
            sections[i] = mode2[i];
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

int check_target_device(uint16_t buttons_values[3]) {
    if (buttons_values[1] == 0) {
        return HYDRO_DOWN;
    } else {
        return HYDRO_UP;
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
    uint16_t buttons_values[3];
    get_button_values_from_message(message, buttons_values);
    if ((!check_target_device(buttons_values)) ||
        (!check_power(buttons_values))) {
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
    uint16_t working_sections[4];
    get_working_sections_by_button_value(buttons_values, working_sections);

    set_pwm_to_sections(sections_pwm, pwm_values, working_sections);

    set_channels_to_sections(section_channel, sections_channels, pwm_values, working_sections);
}
