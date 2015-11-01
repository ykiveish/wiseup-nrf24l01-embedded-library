#ifndef nRFModule_h
#define nRFModule_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "SPInterface.h"

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0        
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

/* Nrf24l settings */
#define ADDR_LEN	5
#define _CONFIG		((1<<EN_CRC) | (0<<CRCO))

typedef void (* funcPtrVoidVoid) ();

typedef enum {
    NRF_250KBPS = 0,
    NRF_1MBPS 	= 1,
    NRF_2MBPS 	= 2,
} speed_rate_t;

typedef enum {
    NRF_0DBM 	= 0,
    NRF_6DBM 	= 1,
    NRF_12DBM 	= 2,
    NRF_18DBM 	= 3,
} power_t;

/**
 * @brief C++ API for NRF24l01 transceiver module
 *
 * This file defines the NRF24l01 C++ interface for libnrf24l01
 *
 * @snippet nrf_receiver.cxx Interesting
 * @snippet nrf_transmitter.cxx Interesting
 */
class NRF24L01 {
	public:
		/**
         * Instanciates a NRF24l01 object
         */
		NRF24L01 (spi_interface* spi_instance);

		/**
         * NRF24l01 object destructor
         */
        ~NRF24L01 ();
		
		/**
         * Initialize needed Gpio pins and SPI interface
         *
         * @param chipSelect setting up the chip select pin
         * @param chipEnable setting up the chip enable pin
         */
		void 	init (uint8_t chipSelect, uint8_t chipEnable);

		/**
         * Configure NRF24l01 chip
         */
		void 	configure ();

		/**
         * Send the buffer data
         *
         * @param *value pointer to the buffer
         */
		void 	send (uint8_t * value);

		/**
         * Send the data located in inner bufer, user must fill the
         * m_txBuffer buffer
         */
		void 	send ();

		/**
         * Set recieving address of the device
         *
         * @param addr 5 bytes addres
         */
		void 	setSourceAddress (uint8_t * addr);

		/**
         * Set recipient address. nrfSend method will send the data buffer
         * to this address
         *
         * @param addr 5 bytes addres
         */
		void 	setDestinationAddress (uint8_t * addr);

		/**
         * Set payload size.
         *
         * @param load size of the payload (MAX 32)
         */
		void 	setPayload (uint8_t load);

		/**
         * Check if data arrived
         */
		bool 	dataReady ();

		/**
         * Check if chip in sending mode
         */
		bool 	dataSending ();

		/**
         * Sink all arrived data into the provided buffer
         *
         * @param load size of the payload (MAX 32)
         */
		void 	getData (uint8_t * data);

		/**
         * Check the chip state
         */
		uint8_t getStatus ();

		/**
         * Check if recieving stack is empty
         */
		bool 	rxFifoEmpty ();

		/**
         * Power up reciever
         */
		void 	rxPowerUp ();

		/**
         * Flush reciver stack
         */
		void 	rxFlushBuffer ();

		/**
         * Power up transmitter
         */
		void 	txPowerUp ();

		/**
         * Power down all
         */
		void 	powerDown ();

		void 	setChannel (uint8_t channel);

		void 	setPower (power_t power);

		uint8_t	setSpeedRate (speed_rate_t rate);

		/**
         * Flush transmit stack
         */
		void 	txFlushBuffer ();

		/**
         * Pulling method which listenning for arrived data, if data
         * arrived dataRecievedHandler will be triggered
         */
		void 	pollListener ();

		/**
         * Set chip enable pin HIGH
         */
		void 	ceHigh ();

		/**
         * Set chip enable LOW
         */
		void 	ceLow ();

		/**
         * Set chip select pin LOW
         */
        void csOn ();

        /**
         * Set chip select pin HIGH
         */
        void csOff ();
		
		/* In sending mode. */
		uint8_t m_ptx;

		/*  CE Pin controls RX / TX, default 8. */
		uint8_t m_cePin;

		/* CSN Pin Chip Select Not, default 7. */
		uint8_t m_csnPin;

		/* Channel 0 - 127 or 0 - 84 in the US. */
		uint8_t m_channel;

		/* Payload width in bytes default 16 max 32. */
		uint8_t m_payload;

		uint8_t m_power;
		
		spi_interface *spi;
		
		uint8_t*		rx_buffer_ptr;
		uint8_t*		tx_buffer_ptr;
		
		funcPtrVoidVoid dataRecievedHandler;
		
		/*
		* Print the NRF registers
		*/
		void printDetails();
		void printByte(char s[],uint8_t reg, uint8_t len, char type);
		void printAddr(char s[],uint8_t reg, uint8_t len);
		
	private:
		/*
		 * Write bytes to the SPI device.
		 */
		void 	writeBytes (uint8_t * dataout, uint8_t * datain, uint8_t len);
		/*
		 * Set register value on SPI device. [one byte]
		 */
		void 	setRegister (uint8_t reg, uint8_t value);
		/*
		 * Get register value from SPI device. [one byte]
		 */
		uint8_t getRegister (uint8_t reg);
		/*
		 * Reads an array of bytes from the given start position in the nrf24l01 registers.
		 */
		void 	readRegister (uint8_t reg, uint8_t * value, uint8_t len);
		/*
		 * Writes an array of bytes into into the nrf24l01 registers.
		 */
		void 	writeRegister (uint8_t reg, uint8_t * value, uint8_t len);
		/*
		 * Send command to the nrf24l01.
		 */
		void 	sendCommand (uint8_t cmd);

		uint8_t bv (uint8_t shift);
};

#endif