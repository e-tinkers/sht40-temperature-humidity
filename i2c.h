#ifndef _I2C_H_
#define _I2C_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <stdbool.h>

#define I2C_WRITE     0
#define I2C_READ      1
#define T_RISE        20UL   // in nano-second, Table 33-19. TWI - Specifications of the datasheet
#define I2C_BUF_SIZE  16

typedef enum {
    SUCCESS,
    BUS_BUSY,
    NACK_RECEIVED_ON_ADDR,
    NACK_RECEIVED_ON_DATA,
    ERROR_OTHERS,
    TIMEOUT
} state_e;

typedef struct {
    PORT_t * port;
    uint8_t scl;
    uint8_t sda;
    uint32_t host_speed;
    uint32_t client_address;
    uint8_t client_send_cnt;
    uint8_t client_rcv_cnt;
    uint8_t * psend_buf;
    uint8_t * prcv_buf;
} i2c_config_t;

volatile uint8_t timeout_cnt;

static inline uint8_t _receivedNack() {
    return TWI0.MSTATUS & TWI_RXACK_bm;
}

static inline void _waitTillCompleted() {
    while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));
}

static inline void _waitTillIdle() {
    while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc));
}

/*
    @brief This function initializes an I2C Host(Master)
    @param i2c the pre-configured i2c object contains settings such as I2C PORT, SCL and SDA pins and host speed
    @return ture - host setup successfully, false - incorrect I2C port specified
*/
bool i2c_host(i2c_config_t * i2c) {
    (i2c->port)->DIRCLR = i2c->scl;          // reset SCL to INPUT high impedance to avoid glitch
    (i2c->port)->OUTCLR = i2c->scl;
    (i2c->port)->OUTSET = i2c->sda;
    if (i2c->port == &PORTB) {              // default port
        (i2c->port)->PIN0CTRL = 0;          // PORTB, PIN0 as SCL
        (i2c->port)->PIN1CTRL = 0;          // PORTB, PIN1 as SDA
    }
 #if defined(PORTMUX_TWI0_bm)               // alternative pins, this does not applicable to 2-series   
    else {
        (i2c->port)->PIN2CTRL = 0;
        (i2c->port)->PIN1CTRL = 0;
        PORTMUX.CTRLB |= PORTMUX_TWI0_bm;        
    }
#else
    else {
        return false;
    }
#endif
    (i2c->port)->OUTSET = i2c->scl;         // set SCL as OUTPUT

    if (i2c->host_speed >= 1000000UL) {
        i2c->host_speed = 1000000UL;        // restrict clock speed to 1MHz
        TWI0.CTRLA |= TWI_FMPEN_bm;         // Enable Fast Mode Plus
    }
    // Equation 26-2 SCL Frequency (page 341 of the datasheet)
    TWI0.MBAUD = (uint8_t) (F_CPU / i2c->host_speed - 10 - (F_CPU * T_RISE / 1000000000UL)) / 2;
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;    // force I2C bus state to IDLE
    TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm | TWI_BUSERR_bm);		// clear flags
    TWI0.CTRLA |= TWI_SDAHOLD_OFF_gc;
    TWI0.MCTRLA |= TWI_ENABLE_bm;

    return true;
}

/*
    @brief This function setup START condition and send i2c address to the slave
    @param address 7-bit i2c address
    @return state_e see state_e definition.
*/
state_e _send_address(uint8_t address, uint8_t mode) {
    // Per 26.3.2.2.3 Transmitting Address Packets of datasheet
    if ((TWI0.MSTATUS & TWI_BUSSTATE_BUSY_gc) == TWI_BUSSTATE_BUSY_gc)
        return BUS_BUSY;

    TWI0.MADDR = address << 1 | mode;

    _waitTillCompleted();

    // m4 case - Arbitration lost or bus error
    if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm))
        return ERROR_OTHERS;

    // m3 case 
    if (_receivedNack()) {
        TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
        _waitTillIdle();
        return NACK_RECEIVED_ON_ADDR;
    }

    // m1 and m2 cases
    return SUCCESS;
}

/*
    @brief This function starts a host WRITE transmission. To READ from a slave, uses i2c_request_from().
    @param address 7-bit i2c address
    @return state_e see state_e definition.
*/
state_e i2c_start(uint8_t address) {
    return _send_address(address, I2C_WRITE);
}

/*
    @brief This function writes data to a Host in Client mode, or queues data for transmission in Host mode.
    @param data pointer to the data buffer
    @param len length of the data to be sent
    @return state_e SUCCESS - data written successfully\n
        NACK_RECEIVED_ON_DATA - slave replied NACK\n
        BUS_BUSY - bus is occupied\n
*/
state_e i2c_write(uint8_t *data, size_t len) {

    size_t count = 0;

    if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == TWI_BUSSTATE_OWNER_gc) {
        while (count < len) {
            TWI0.MDATA = data[count++];
            _waitTillCompleted();
            if (_receivedNack())
                return NACK_RECEIVED_ON_DATA;
        }
    }
    else {
        return BUS_BUSY;
    }

    return SUCCESS;
}

/*
    @brief This function send STOP condition and wait till bus is IDLE
    @param none
    @return none
*/
void i2c_stop(void) {
  TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
  _waitTillIdle();
}

/*
    @brief This function sending the i2c address with READ request to the slave
    @param address 7-bit I2C address
    @return none
*/
state_e i2c_request_from(uint8_t address) {
    return _send_address(address, I2C_READ);
}

/*
    @brief This function read data from I2C peripheral.
    @param data pointer to the receiving data buffer
    @param len length of data to receive from
    @return none
*/
void i2c_read(uint8_t  *data, size_t len) {

  size_t count = 0;
  
  TWI0.MSTATUS = TWI_CLKHOLD_bm;          // release the clock hold
  
  while (count < len) {
    _waitTillCompleted();
    data[count++] = TWI0.MDATA;
    if (count != len)
        TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc; // if ACK = more bytes to read
  }
  TWI0.MCTRLB = TWI_ACKACT_bm |  TWI_MCMD_STOP_gc;; // else Send NAK and STOP
  _waitTillIdle();
  
}

#endif