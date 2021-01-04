#pragma once
#include "core.h"

// available modes
uint16_t available_modes[2] = {0, 1};
uint16_t PWM_THRESHOLD = 30; // PWM threshold value. If less threshold then zero 

uint16_t working_sections_table[4][4] = { // i - mode, j - section index
    0, 1, 2, 3,
    0, 1, 4, 5,
    6, 7, 8, 9,
    6, 7, 10, 11
};

uint16_t working_modes_table[2][2] = { // i - button 1 value, j - button 2 value
    0, 2,
    1, 3
};

int check_mode_availability(uint16_t val, uint16_t arr[], int number_of_elements){
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

void get_mode_from_button_values(uint16_t button_values[], uint16_t *mode) {
    *mode = working_modes_table[button_values[1]][button_values[2]];
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

int check_action_permision(uint16_t buttons_values[3]) {
  return buttons_values[0];
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

void get_button_values_from_message(uint16_t message[],
                                    uint16_t buttons_values[3]) {
    buttons_values[0] = message[4];
    buttons_values[1] = message[5];
    buttons_values[2] = message[6];
}

void start_engine(uint16_t buttons_values[],
                  uint16_t non_hydraulic_actions[]) {
    if (buttons_values[0] == 1) {
        non_hydraulic_actions[0] = 1;
    }
}

void stop_engine(uint16_t buttons_values[],
                 uint16_t non_hydraulic_actions[]) {
    if (buttons_values[0] == 0) {
        non_hydraulic_actions[0] = 0;
    }
}

void turn_on_beep(uint16_t buttons_values[],
                  uint16_t non_hydraulic_actions[]) {
    if (buttons_values[0] == 1) {
        non_hydraulic_actions[1] = 1;
    }
}

void turn_off_beep(uint16_t buttons_values[],
                   uint16_t non_hydraulic_actions[]) {
    if (buttons_values[0] == 0) {
        non_hydraulic_actions[1] = 0;
    }
}

void turn_on_light(uint16_t buttons_values[],
                   uint16_t non_hydraulic_actions[]) {
    if (buttons_values[0] == 1) {
        non_hydraulic_actions[2] = 1;
    }
}

void turn_off_light(uint16_t buttons_values[],
                    uint16_t non_hydraulic_actions[]) {
    if (buttons_values[0] == 0) {
        non_hydraulic_actions[2] = 0;
    }
}

void handle_message(
    uint16_t zero_message[],
    uint16_t message[],
    uint16_t sections_pwm[],
    uint16_t* section_channel,
    uint16_t non_hydraulic_actions[]
) {
    //
    uint16_t buttons_values[3];
    get_button_values_from_message(message, buttons_values);
    // Block actions with robot
    if (!check_action_permision(buttons_values)) {
        return;
    }
    // handle non-hydralic functions
    start_engine(buttons_values, non_hydraulic_actions);
    stop_engine(buttons_values, non_hydraulic_actions);

    turn_on_beep(buttons_values, non_hydraulic_actions);
    turn_off_beep(buttons_values, non_hydraulic_actions);

    turn_on_light(buttons_values, non_hydraulic_actions);
    turn_off_light(buttons_values, non_hydraulic_actions);

    // handle hydralic functions
    int mode = 65535;
    get_mode_from_button_values(buttons_values, &mode);
    if (!check_mode_availability(mode, available_modes, 2)) {
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
    //Set pwm values to working sections
    set_pwm_to_sections(sections_pwm, pwm_values, working_sections_table[mode]);
    //Set channel values to working sections
    set_channels_to_sections(section_channel, sections_channels, pwm_values, working_sections_table[mode]);
}
