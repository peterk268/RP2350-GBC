void read_volume(i2s_config_t *i2s_config) {
    // ADC reference voltage (assumed 3.3V)
    const float adc_ref_voltage = 3.3f;

    // Read Headphone Detect ADC Value
    adc_select_input(GPIO_nHP_DETECT - 40); // Map GPIO to ADC channel
    uint16_t hp_detect_adc_value = read_adc();

    // Convert raw ADC value to voltage
    float hp_detect_voltage = (hp_detect_adc_value / 4095.0f) * adc_ref_voltage;

    // Read Volume Potentiometer ADC Value
    adc_select_input(GPIO_AUD_POT_ADC - 40); // Map GPIO to ADC channel
    uint16_t volume_adc_value = read_adc();

    float volume_level = (float)volume_adc_value / 4095.0f;  // This scales adc_value to a range from 0 to 1
    printf("Volume Level (0-1): %.2f, ADC Value: %d\n", volume_level, volume_adc_value);

    // Scale the volume based on headphone detect voltage
    if (hp_detect_voltage < 3.0f) { // Check if voltage is less than 3V (headphones present)
        i2s_volume(i2s_config, (volume_level) / 2.0f); // Reduced volume scaling
    } else {
        i2s_volume(i2s_config, volume_level); // Full volume
    }
}
