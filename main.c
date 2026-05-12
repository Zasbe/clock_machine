// First implementation of a 4 tempo synced outputs clock generator in C using the default
// hardware timer and the 4 alarm IRQs with indipendent timeout setting.
// Also it utilizes a fixed step voltage generator via a DAC, configured with SPI and
// synced along with the full-note output.

#include "pico/stdio.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "ssd1306.h"
#include "font.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

//LED ouputs pins
#define led_power_on 25
#define led_mode 15
#define led_quarter_note 27
#define led_eighth_note 21
#define led_sixtnth_note 20
#define led_tap 9

//TAP TEMPO and MODE button pins
const uint tap_button = 10;
const uint mode_button = 14;

//I2C pins
#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13
//Display update flag
volatile bool display_up = false;

// ADC pin
#define ADC 26
#define ADC_AVERAGE 64

uint16_t adc_avg_read (void){
    uint32_t sum = 0;
    for (int i = 0 ; i < ADC_AVERAGE; i++) {
        sum += adc_read();
    }
    return (uint16_t)(sum/ADC_AVERAGE);
}

// Rotary encoder pin inputs
#define ENC_A 2
#define ENC_B 3
volatile int enc_count = 0;

//SPI pin declarations
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define PIN_LDAC 28

//SPI Chip Select pin control functions declaration
void cs_select(){
    gpio_put(PIN_CS, 0);
}
void cs_deselect(){
    gpio_put(PIN_CS, 1);
}

//DAC configuration word construction
//DAC write function with SPI write16 on the specified SPI instance
#define DAC_CHA 0X0000
#define DAC_CHB 0X8000
#define DAC_DNTC 0X0000
#define DAC_GAIN_X1 0X2000
#define DAC_GAIN_X2 0X0000
#define DAC_ACTIVE 0X1000

void dac_write(uint8_t channel, uint16_t value){
    uint16_t confg_word = ((channel == 0) ? DAC_CHA : DAC_CHB)
        | DAC_DNTC
        | DAC_GAIN_X1
        | DAC_ACTIVE
        | (value & 0x0fff);

    cs_select();
    spi_write16_blocking(SPI_PORT, &confg_word, 1);
    cs_deselect();
}

//Just a toggle function with sleep ms
void toggle_out(uint pin, uint ms){
    gpio_put(pin, 1);
    sleep_ms(ms);
    gpio_put(pin, 0);
    sleep_ms(ms);
}

//Definisions for BPM calculation
#define TAP_BUFFER_SIZE 8
#define BPM_MIN 60
#define BPM_MAX 221
#define PERIOD_MIN ((60000000ULL) / BPM_MAX)
#define PERIOD_MAX ((60000000ULL) / BPM_MIN)

volatile uint64_t BPM;
volatile uint64_t period;
volatile uint64_t ch1_period;
volatile uint64_t ch2_period;
volatile uint64_t ch3_period;
volatile uint64_t new_tick_us = 0;
volatile uint64_t old_tick_us = 0;
volatile uint64_t time_interval = 0;
volatile uint64_t tap_intervals[TAP_BUFFER_SIZE]={0};
volatile uint tap_index = 0;
volatile uint8_t tap_counter = 0;
volatile bool resync = false;

//GPIO interrupt flags
volatile bool tap_flag = false;
volatile bool mode_flag = false;
volatile uint8_t mode_select = 0;

//GPIO interrupt handlers per button
// TAP MODE and BPM calculation logic
void on_tap(uint32_t events){
    tap_flag = gpio_get(tap_button) == 0;

    if (gpio_get(tap_button) != 0 )
        return;
    if (mode_select !=0) {
        return;
    }

    new_tick_us = time_us_64();

    if (old_tick_us != 0){
        time_interval = new_tick_us - old_tick_us;

        if (time_interval > 2000000){ // check if 2s passed without any tap
            tap_index = 0;
            tap_counter = 0;
        }
        else {
            tap_intervals[tap_index] = time_interval;
            tap_index = (tap_index + 1) % TAP_BUFFER_SIZE;

            if (tap_counter < TAP_BUFFER_SIZE) tap_counter++;

            uint64_t sum = 0;
            for (uint8_t i = 0; i < tap_counter; i++){
                sum += tap_intervals[i];
            }
            uint64_t raw_period = sum/tap_counter;
            BPM = (60000000ULL) / raw_period; //updated BPM valuew to disply

            if (BPM < BPM_MIN)
                BPM = BPM_MIN;
            if (BPM > BPM_MAX)
                BPM = BPM_MAX;

            period = 60000000ULL / BPM;

            resync = true;
            display_up = true;
        }
    }
    old_tick_us = new_tick_us;
}

void on_mode(uint32_t events){
    mode_flag = gpio_get(mode_button)== 0;
    if (gpio_get(mode_button) != 0){
        return;
    }
    ++mode_select;
    if (mode_select > 1) {
        mode_select = 0;
    }
    tap_index = 0;
    tap_counter = 0;
    old_tick_us = 0;  // ← critical

    display_up = true;
}

void enc_read_a (uint32_t events){
    bool check = gpio_get(ENC_B);
    if (events == GPIO_IRQ_EDGE_FALL){
        if (check) {
            ++BPM;
            period = 60000000/BPM;
            resync = true;
            display_up = true;
        }
        else {
            --BPM;
            period = 60000000/BPM;
            resync = true;
            display_up = true;
        }
    }
    if (events == GPIO_IRQ_EDGE_RISE){
        if (!check) {
            ++BPM;
            period = 60000000/BPM;
            resync = true;
            display_up = true;
        }
        else {
            --BPM;
            period = 60000000/BPM;
            resync = true;
            display_up = true;
        }
    }
}

void enc_read_b (uint32_t events){
    bool check = gpio_get(ENC_A);
    if (events == GPIO_IRQ_EDGE_FALL){
        if (check) {
            --BPM;
            period = 60000000/BPM;
            resync = true;
            display_up = true;
        }
        else {
            ++BPM;
            period = 60000000/BPM;
            resync = true;
            display_up = true;
        }
    }
    if (events == GPIO_IRQ_EDGE_RISE){
        if (!check) {
            --BPM;
            period = 60000000/BPM;
            resync = true;
            display_up = true;
        }
        else {
            ++BPM;
            period = 60000000/BPM;
            resync = true;
            display_up = true;
        }
    }
}

//GPIO callback function definition
void gpio_interrupt_fire (uint gpio, uint32_t events){
    switch (gpio) {
        case tap_button: on_tap(events); break;
        case mode_button: on_mode(events); break;
        case ENC_A: enc_read_a(events); break;
        case ENC_B: enc_read_b(events); break;
    }
}

//Timer checks definitions
volatile uint64_t new_tick = 0;
volatile uint64_t old_tick = 0;

//Repeating Timer callback definition
bool timer_callback (struct repeating_timer *t){
    return true;
}

volatile bool alarm1_flag = false;
volatile absolute_time_t alarm1_target;
void alarm_1(uint alarm_num){
    // printf("alarm 1 triggered, %llu\n", time_us_64());
    // hardware_alarm_cancel(0);
    gpio_put(led_quarter_note, alarm1_flag);
    alarm1_flag = !alarm1_flag;
    alarm1_target = delayed_by_us(alarm1_target, ch1_period);
    hardware_alarm_set_target(0, alarm1_target);
}

volatile bool alarm2_flag = false;
volatile absolute_time_t alarm2_target;
void alarm_2(uint alarm_num){
    // printf("alarm 2 triggered, %llu\n", time_us_64());
    // hardware_alarm_cancel(1);
    gpio_put(led_eighth_note, alarm2_flag);
    alarm2_flag = !alarm2_flag;
    alarm2_target = delayed_by_us(alarm2_target, ch2_period);
    hardware_alarm_set_target(1, alarm2_target);
}

volatile bool alarm3_flag = false;
volatile absolute_time_t alarm3_target;
void alarm_3(uint alarm_num){
    // printf("alarm 3 triggered, %llu\n", time_us_64());
    // hardware_alarm_cancel(2);
    gpio_put(led_sixtnth_note, alarm3_flag);
    alarm3_flag = !alarm3_flag;
    alarm3_target = delayed_by_us(alarm3_target, ch3_period);
    hardware_alarm_set_target(2, alarm3_target);
}

//Main function entry point
int main(void){
    stdio_init_all();//USB and UART init
    busy_wait_ms(2000);

    //I2C Init
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);
    gpio_pull_up(I2C_SDA);

    busy_wait_ms(500);

    //Display init
    ssd1306_t display;
    ssd1306_init(&display, 128, 64, 0x3C, I2C_PORT);
    ssd1306_clear(&display);

    // Initial BPM setting
    BPM = 120;
    period = (60000000ULL)/BPM;

    // Initial display show from power up
    char buffer[16];
    char mode_buf[16];
    sniprintf(mode_buf, sizeof(mode_buf), "*TAP mode*");
    sniprintf(buffer, sizeof(buffer), "BPM:%llu", BPM);
    ssd1306_draw_string(&display, 0, 1, 2, mode_buf);
    ssd1306_draw_string(&display, 0, 28, 2, buffer);
    ssd1306_show(&display);

    static uint32_t last_adc_check = 0;
    uint32_t now = 0;

    // ADC init
    adc_init();
    adc_gpio_init(ADC);
    adc_select_input(0);
    gpio_disable_pulls(ADC);

    //SPI init and pin function control
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    //SPI Chip Select pin init
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    cs_deselect();

    //Power on led init
    gpio_init(led_power_on);
    gpio_set_dir(led_power_on, GPIO_OUT);
    //Power indicator led set to on
    gpio_put(led_power_on, 1);

    //LED outputs init
    gpio_init(led_mode);
    gpio_set_dir(led_mode, GPIO_OUT);
    gpio_init(led_quarter_note);
    gpio_set_dir(led_quarter_note, GPIO_OUT);
    gpio_init(led_eighth_note);
    gpio_set_dir(led_eighth_note, GPIO_OUT);
    gpio_init(led_sixtnth_note);
    gpio_set_dir(led_sixtnth_note, GPIO_OUT);
    gpio_init(led_tap);
    gpio_set_dir(led_tap, GPIO_OUT);

    //TAP TEMPO pin init
    gpio_init(tap_button);
    gpio_set_pulls(tap_button, 1, 0);
    gpio_set_dir(tap_button, GPIO_IN);

    //MODE Pin init
    gpio_init(mode_button);
    gpio_set_pulls(mode_button, 1, 0);
    gpio_set_dir(mode_button, GPIO_IN);

    // Encoder pins init
    gpio_init(ENC_A);
    gpio_disable_pulls(ENC_A);
    gpio_set_dir(ENC_A, GPIO_IN);

    gpio_init(ENC_B);
    gpio_disable_pulls(ENC_B);
    gpio_set_dir(ENC_B, GPIO_IN);

    //GPIO Interrupts enabled and callback
    gpio_set_irq_enabled(tap_button, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(mode_button, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(ENC_A, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(ENC_B, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_callback(gpio_interrupt_fire);
    irq_set_enabled(IO_IRQ_BANK0, true);

    //Repeating Timer init
    struct repeating_timer timer1;
    add_repeating_timer_us(period >> 2, timer_callback, NULL, &timer1);

    // Alarm setings and init
    //shared base time for coherent phase
    ch1_period = period >> 1;
    ch2_period = period >> 2;
    ch3_period = period >> 3;

    absolute_time_t base = delayed_by_us(get_absolute_time(), 100);

    //ALARM interrupt 1
    hardware_alarm_claim(0);
    hardware_alarm_set_callback(0, alarm_1);

    //ALARM interrupt 2
    hardware_alarm_claim(1);
    hardware_alarm_set_callback(1, alarm_2);

    //ALARM interrupt 3
    hardware_alarm_claim(2);
    hardware_alarm_set_callback(2, alarm_3);

    alarm1_target = delayed_by_us(base, ch1_period);
    alarm2_target = delayed_by_us(base, ch2_period + ch1_period);
    alarm3_target = delayed_by_us(base, ch3_period + ch2_period + ch1_period);

    hardware_alarm_set_target(0, alarm1_target);
    hardware_alarm_set_target(1, alarm2_target);
    hardware_alarm_set_target(2, alarm3_target);

    // Mode select init variables
    volatile uint16_t new_raw = 0;
    volatile uint16_t old_raw = 0;

    while (1){
        gpio_put(led_tap, tap_flag);
        gpio_put(led_mode, mode_flag);

        if (mode_select == 1) {
            now = time_us_32();
            if (now - last_adc_check > 50000) {
                last_adc_check = now;
                new_raw = adc_avg_read();
                int dif =(new_raw - old_raw);
                if (abs(dif) > 5) {
                    BPM = BPM_MIN + (new_raw * (BPM_MAX - BPM_MIN)) / 4095;
                    period = 60000000ULL / BPM;
                    resync = true;
                    display_up = true;
                    old_raw = new_raw;
                }
            }
        }

        if (resync && gpio_get(led_quarter_note) == 0 && gpio_get(led_eighth_note) == 0 && gpio_get(led_sixtnth_note) == 0){

            hardware_alarm_cancel(0);
            hardware_alarm_cancel(1);
            hardware_alarm_cancel(2);

            alarm1_flag = false;
            alarm2_flag = false;
            alarm3_flag = false;

            gpio_put(led_quarter_note, 0);
            gpio_put(led_eighth_note, 0);
            gpio_put(led_sixtnth_note, 0);

            ch1_period = period >> 1;
            ch2_period = period >> 2;
            ch3_period = period >> 3;

            base = delayed_by_us(get_absolute_time(),1);
            alarm1_target = delayed_by_us(base, ch1_period);
            alarm2_target = delayed_by_us(base, ch2_period + ch1_period);
            alarm3_target = delayed_by_us(base, ch3_period + ch2_period + ch1_period);

            hardware_alarm_set_target(0, alarm1_target);
            hardware_alarm_set_target(1, alarm2_target);
            hardware_alarm_set_target(2, alarm3_target);

            resync = false;
        }

        if (display_up) {
            display_up = false;
            char buffer[16];
            char mode_buf [16];

            ssd1306_clear(&display);

            if (mode_select == 0) {
                 sniprintf(mode_buf, sizeof(mode_buf), "*TAP mode*");
                 sniprintf(buffer, sizeof(buffer), "BPM:%llu", BPM);
                 ssd1306_draw_string(&display, 0, 1, 2, mode_buf);
                 ssd1306_draw_string(&display, 0, 28, 2, buffer);
            }

            else if (mode_select == 1) {
                 sniprintf(mode_buf, sizeof(mode_buf), "*INT mode*");
                 sniprintf(buffer, sizeof(buffer), "BPM:%llu", BPM);
                 ssd1306_draw_string(&display, 0, 1, 2, mode_buf);
                 ssd1306_draw_string(&display, 0, 28, 2, buffer);
            }

            ssd1306_show(&display);
        }
    }
}
