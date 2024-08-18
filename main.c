#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "lcd1602.h"
#include "i2c.h"

// I2C Host clock speed
#define I2C_CLOCK_SPEED  100000UL

// SHT40 
#define SHT40_ADDRESS    0x44
#define SHT40_READ_TEMP  0xFD

// Timeer configuration
#define WAKEUP_INTERVAL  59

// Custom Characters
#define SYM_THERMOMITOR 1
#define SYM_HUMIDITY    2
#define SYM_BATTERY     3

// adc parameters
#define TIMEBASE_VALUE 20     // ceil(F_CPU*0.000001) i.e. number of clock cycles to reach 1uS
#define ADC_MAX_STEP   4095   // In single-ended mode, the max value is 4095
#define ADC_REF        2.500F

// define the port and pins used by the LCD1602
LCD_t lcd = {
    .port = &PORTA,
    .rs = PIN2_bm,
    .en = PIN3_bm,
    .d4 = PIN4_bm,
    .d5 = PIN5_bm,
    .d6 = PIN6_bm,
    .d7 = PIN7_bm,
};

// I2C device configuration
i2c_config_t i2c= {
    .port = &PORTB,
    .scl = PIN0_bm,
    .sda = PIN1_bm,
    .host_speed = I2C_CLOCK_SPEED,
    .client_address = SHT40_ADDRESS
};

volatile uint32_t timeCount = 0;

ISR (RTC_PIT_vect) {
    timeCount++;
    RTC.PITINTFLAGS = RTC_PI_bm;  // Clear periodic interrupt flag
}

/* Using external crystal doesn't see significant difference from using internal clock */
void configRTC() {

  /* Using ULP internal 32.768kHz clcok */
  while (RTC.STATUS > 0);
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;                    // 32.768kHz Internal Oscillator
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm; // Interrrupt every 1 second
  RTC.PITINTCTRL = RTC_PI_bm;

}

void adc_init() {

    ADC0.CTRLB = ADC_PRESC_DIV16_gc;         // fCLK_ADC = 20000000/16 = 1250000 Hz
    ADC0.CTRLC = (TIMEBASE_VALUE << ADC_TIMEBASE_gp) | ADC_REFSEL_2500MV_gc; // timebase value count & ref voltage source
    ADC0.CTRLE = 17;                         // Sample duration=(17 + 0.5) / fCLK_ADC * 1000000=14uS for non-PGA
    ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;        // AIN1 = PA1
    ADC0.COMMAND = ADC_MODE_SINGLE_12BIT_gc; // Single 12-bit mode
    ADC0.CTRLA = ADC_ENABLE_bm;
}

void turnOffUnusedPins() {
  // set all pin to input
  PORTA.DIRCLR = PIN0_bm | PIN1_bm;
  PORTB.DIRCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;

  PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;

  PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

}

int main() {

    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, !CLKCTRL_PEN_bm); // disable prescaler to run at 20MHz
    while (!(CLKCTRL.MCLKSTATUS & CLKCTRL_OSC20MS_bm)) {};
    
    turnOffUnusedPins();

    configRTC();

    lcd_config();
    lcd_init(LCD_CURSOR_OFF, LCD_BLINK_OFF);
    lcd_create_char(SYM_THERMOMITOR, thermomitorSym);
    lcd_create_char(SYM_HUMIDITY, humiditySym);
    lcd_create_char(SYM_BATTERY, batterySym);

    i2c_host(&i2c);

    SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc;  // config sleep controller powerdown mode
    sei();

    while(1) {
        if (timeCount == 0 || timeCount > WAKEUP_INTERVAL) {

            timeCount = 0;

            uint8_t reg_temp_high_precision[] = {SHT40_READ_TEMP};
            i2c_start(i2c.client_address);
            i2c_write(reg_temp_high_precision, 1);
            i2c_stop();

            // measurement duration for High Repeatability is 6.9ms typical, 8.3ms maximum
            // as per SHT40 datasheet
            _delay_ms(9);

            uint8_t raw[6] = {0};
            i2c_request_from(SHT40_ADDRESS);
            i2c_read(raw, sizeof(raw));

            // convert to Temperature/Humidity
            double temperature = -45 + 175.0 * ((uint16_t)raw[0] << 8 | raw[1]) / 65535;
            double humidity = -6 + 125.0 * ((uint16_t)raw[3] << 8 | raw[4]) / 65535;

            // read battery voltage
            adc_init();                   // config and adc
            ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;
            while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));
            float adc_volt = (float)(ADC0.SAMPLE) / ADC_MAX_STEP * ADC_REF;
            float vBatt = adc_volt * 2;   // reistors of voltage divider has equal value
            ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable ADC

            BOD.CTRLA = BOD_SLEEP_DIS_gc; // turn off brown out detection during sleep
            
            // print data to LCD
            char msg[16] = {0};
            sprintf(msg, "%c %.1f %cC", (char) SYM_THERMOMITOR, temperature, (char) 0xdf);
            lcd_cursor(1, 1);
            lcd_print_str(msg);

            sprintf(msg, "%c %.1f %%  %c %.1fv", (char) SYM_HUMIDITY, humidity, (char)SYM_BATTERY, vBatt);
            lcd_cursor(2, 1);
            lcd_print_str(msg);
        }
        SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;  // sleep enable
        __asm("sleep");                   // go to sleep mode
        SLPCTRL.CTRLA &= ~SLPCTRL_SEN_bm; // sleep disable
    }

    return 0;

}
