#include "logic.h"


int get_pwm_from_input_data(uint16_t input_data, int zero_value) {
    if (input_data >= zero_value) {
        return (input_data - zero_value);
    } else {
        return (zero_value - input_data);
    }
}

int get_channel_from_input_data(uint16_t input_data, int zero_value) {
    if (input_data >= zero_value) {
        return 0;
    } else {
        return 1;
    }
}

void set_and_reset_channel(uint16_t *section_channel, int channel_counter, int channel) {
    if (channel) {
        // set 1 channel
        *section_channel |= ((1 << channel_counter) << 1);
        // reset 0 channel
        *section_channel &= ~((1 << channel_counter) << 0);
    } else {
        // set 0 channel
        *section_channel |= ((1 << channel_counter) << 0);
        // reset 1 channel
        *section_channel &= ~((1 << channel_counter) << 1);
    }
}

int threshold_pwm(int pwm, int threshold) {
  if (pwm > threshold) {
    return pwm;
  } else {
    return 0;
  }
}

void foo(int *worked_sections,
         int worked_sections_size,
         int potenciometers_values[],
         int zero_potenciometers_values[],
         uint16_t sections_pwm[],
         uint16_t* section_channel) {
    int channel_counter = 0;

    for (int i=0; i < worked_sections_size; i++) {
        int section_index = worked_sections[i];

        channel_counter = (section_index * 2);

        int section_pwm = get_pwm_from_input_data(potenciometers_values[i],
                                                  zero_potenciometers_values[i]);
        
        sections_pwm[section_index] = threshold_pwm(section_pwm, 30);

        int channel = get_channel_from_input_data(potenciometers_values[i],
                                                  zero_potenciometers_values[i]);

        set_and_reset_channel(section_channel, channel_counter, channel);
    }
}

void parse_section_pwm_and_channel_from_message(
    uint16_t zero_message[],
    uint16_t message[],
    uint16_t sections_pwm[],
    uint16_t* section_channel
) {
    int buttons_values[2];
    int potenciometers_values[4];
    int zero_potenciometers_values[4];
    for (int i = 0; i < 4; i++) {
        potenciometers_values[i] = message[i];
        zero_potenciometers_values[i] = zero_message[i];
    }
    buttons_values[0] = message[4];
    buttons_values[1] = message[5];

    int mode1[] = {0, 1, 2, 3};
    int mode2[] = {4, 5};

    if (buttons_values[0]) {
        //mode1
        int mode1_size = sizeof(mode1) / sizeof(mode1[0]);
        foo(mode1,
            mode1_size,
            potenciometers_values,
            zero_potenciometers_values,
            sections_pwm,
            section_channel);
    } else {
        //mode2
        int mode2_size = sizeof(mode2) / sizeof(mode2[0]);
        foo(mode2,
            mode2_size,
            potenciometers_values,
            zero_potenciometers_values,
            sections_pwm,
            section_channel);
    }
    
}
