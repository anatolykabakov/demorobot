
int get_pwm_from_input_data(unsigned short input_data, int zero_value) {
    if (input_data >= zero_value) {
        return (input_data - zero_value);
    } else {
        return (zero_value - input_data);
    }
}
//определить канал по значению потенциомента
int get_channel_from_input_data(unsigned short input_data, int zero_value) {
    if (input_data >= zero_value) {
        return 0;
    } else {
        return 1;
    }
}
/*!
  Description of parse_section_pwm_and_channel_from_message

  \param[in] message - unsigned char[] description
  struct message: {JoysticLeftXAxis,
                   JoysticLeftYAxis,
                   JoysticRightXAxis
                   JoysticRightYAxis,
                   button1State,
                   button2State}
    unsigned short message[],
    unsigned short* section_channel,
    unsigned short section_pwm[]
) {

}
  \param[out] section_channel - unsigned short* description
  \param[out] section_pwm - unsigned short[] description

  */
void parse_section_pwm_and_channel_from_message(
    unsigned short zero_message[],
    unsigned short message[],
    unsigned short sections_pwm[],
    unsigned short* section_channel
) {
    /*
      1. распарить на значения потенциоментов и кнопок
      2. определить какие секции работают по состоянию кнопок
      3. нормировать значения потенциометра относительно нуля
      4. определить какие каналы секций включать и какие выключать, 
         для данных значений джойстиков
      5. для рабочих секций установить значения каналов
    */
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

void set_and_reset_channel(int *section_channel, int channel_counter, int channel) {
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

void foo(int *worked_sections,
         int worked_sections_size,
         int potenciometers_values[],
         int zero_potenciometers_values[],
         unsigned short sections_pwm[],
         unsigned short* section_channel) {
    int channel_counter = 0;

    for (int i=0; i < worked_sections_size; i++) {
        int section_index = worked_sections[i];

        channel_counter = (section_index * 2);

        sections_pwm[i] = get_pwm_from_input_data(potenciometers_values[i],
                                                  zero_potenciometers_values[i]);

        int channel = get_channel_from_input_data(potenciometers_values[i],
                                                  zero_potenciometers_values[i]);

        set_and_reset_channel(section_channel, channel_counter, channel);
    }
}
