//
// FEATURE BRANCH -- ANALOG-TO-DIGITAL CONVERSION
// WHEN DONE REFACTOR TO INDEPENDENT ADC MODULE
//

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "aery32/gpio.h"
#include "aery32/spi.h"
#include "aery32/rtc.h"
#include "board.h"


// ----------------------------------------------------------------------
// HD44780 instruction set
// ----------------------------------------------------------------------
#define HD44780_CLEAR_DISPLAY 0x01
#define HD44780_RETURN_HOME 0x02

#define HD44780_EMODE_INCREMENT 0x06
#define HD44780_EMODE_DECREMENT 0x04
#define HD44780_EMODE_INCRNSHIFT 0x07 // Increment and shift
#define HD44780_EMODE_DECRNSHIFT 0x05 // Decrement and shift

#define HD44780_DISPLAY_ON 0x0C
#define HD44780_DISPLAY_OFF 0x08
#define HD44780_CURSOR_ON 0x0E
#define HD44780_CURSOR_ONBLINK 0x0F

#define HD44780_LSHIFT_CURSOR 0x10
#define HD44780_RSHIFT_CURSOR 0x14
#define HD44780_LSHIFT_DISPLAY 0x18
#define HD44780_RSHIFT_DISPLAY 0x1C

#define HD44780_DL4BIT 0x28
#define HD44780_DL8BIT 0x38
#define HD44780_FONTBL_ENJA 0x28
#define HD44780_FONTBL_WE1 0x29
#define HD44780_FONTBL_ENRU 0x2A
#define HD44780_FONTBL_WE2 0x2B

#define HD44780_DDRAM_ADDR 0x80
#define HD44780_CGRAM_ADDR 0x40

#define HD44780_BUSYBIT_MASK 0x80


// ----------------------------------------------------------------------
// Board settings
// ----------------------------------------------------------------------
#define LED AVR32_PIN_PC04
#define SPI0_GPIO_MASK ((1 << 10) | (1 << 11) | (1 << 12) | (1 << 13))

#define DISPLAY_SPI (&AVR32_SPI0)
#define DISPLAY_SPI_NPCS 0
#define DISPLAY_SPI_MODE SPI_MODE3


// ----------------------------------------------------------------------
// API to operate with the display
// ----------------------------------------------------------------------
#define MAX_DISPBUF 21
char dispbuf[MAX_DISPBUF] = {'\0'};

bool display_isbusy(void);
void display_instruct(uint16_t);
void display_wrbyte(uint8_t);
void display_putc(char);
uint8_t display_puts(const char*);
int display_printf(const char*, ... );


// ----------------------------------------------------------------------
// ADC
// ----------------------------------------------------------------------
volatile avr32_adc_t *adc = &AVR32_ADC;
#define ADC_VREF 3.0

uint16_t adc_read(uint8_t chanum)
{
	return *(&(AVR32_ADC.cdr0) + (chanum * 4));
}

double adc_conv2volts(uint16_t conv, double vref)
{
	return (vref / 1024) * conv;
}


// ----------------------------------------------------------------------
// Main function
// ----------------------------------------------------------------------
int
main(void)
{
	int i;
	uint16_t c[8] = {0}; // conversion result table

	init_board();
	aery_gpio_init_pin(LED, GPIO_OUTPUT);
	aery_gpio_init_pins(porta, SPI0_GPIO_MASK, GPIO_FUNCTION_A);
	aery_gpio_init_pins(porta, 0xff << 21, GPIO_FUNCTION_A);

	// Init RTC for delays
	aery_gpio_enable_localbus();
	aery_rtc_init(0, 0xffffffff, 0, RTC_SOURCE_RC);
	aery_rtc_enable(false);

	// Init SPI0 for display
	aery_spi_init_master(DISPLAY_SPI);
	aery_spi_setup_npcs(DISPLAY_SPI, DISPLAY_SPI_NPCS, DISPLAY_SPI_MODE, 10);
	aery_spi_enable(DISPLAY_SPI);

	// Screen initialization sequence
	for (uint32_t i = 0; i < 1000000; i++); // wait > 1ms
	spi0->CSR0.scbr = 0xff; // underclock spi bus, baudrate = MCK/255
	display_instruct(HD44780_DL8BIT|HD44780_FONTBL_WE1);
	display_instruct(HD44780_DISPLAY_OFF);
	display_instruct(HD44780_CLEAR_DISPLAY);
	display_instruct(HD44780_RETURN_HOME);
	display_instruct(HD44780_EMODE_INCREMENT);
	display_instruct(HD44780_CURSOR_ONBLINK);

	// ADC initialization sequence
	adc->MR.prescal = 0; // adc_clock = clk_adc / (prescal+1) / 2
	adc->MR.shtim = 0; // sample and hold time, relative to adc_clock
	adc->MR.startup = 0; // start up time, relative to adc_clock
	adc->MR.sleep = 0; // no sleep, use normal mode
	adc->MR.lowres = 0; // 10-bit resolution, 1 would have been 8-bit
	adc->MR.trgen = 0; // use software start for conversion

	adc->cher = 0xff; // enable all channels

	// Init OK
	aery_gpio_set_pin_high(LED);

	for(;;) {
		/* Put your application code here */

		// Start conversion
		adc->CR.start = true;

		// Wait conversion to complete for all channels
		while ((adc->sr & 0xff) != 0xff);

		// Collect conversion data
		for (i = 0; i < 8; i++) {
			c[i] = adc_read(i);
		}

		// Show conversion result
		display_instruct(HD44780_DDRAM_ADDR);
		display_printf("1. %.2f 2. %.2f",
			adc_conv2volts(c[0], ADC_VREF),
			adc_conv2volts(c[1], ADC_VREF)
		);
		display_instruct(0x40|HD44780_DDRAM_ADDR);
		display_printf("3. %.2f 4. %.2f",
			adc_conv2volts(c[2], ADC_VREF),
			adc_conv2volts(c[3], ADC_VREF)
		);

		aery_rtc_delay_cycle(57500);

		display_instruct(HD44780_DDRAM_ADDR);
		display_printf("5. %.2f 6. %.2f",
			adc_conv2volts(c[4], ADC_VREF),
			adc_conv2volts(c[5], ADC_VREF)
		);
		display_instruct(0x40|HD44780_DDRAM_ADDR);
		display_printf("7. %.2f 8. %.2f",
			adc_conv2volts(c[6], ADC_VREF),
			adc_conv2volts(c[7], ADC_VREF)
		);

		aery_rtc_delay_cycle(57500);
	}

	return 0;
}


// ----------------------------------------------------------------------
// Display functions
// ----------------------------------------------------------------------
bool
display_isbusy(void)
{
	uint16_t rd; /* read data */

	aery_spi_transmit(DISPLAY_SPI, 0x100, DISPLAY_SPI_NPCS, false);
	rd = aery_spi_transmit(DISPLAY_SPI, 0x100, DISPLAY_SPI_NPCS, true) >> 2;
	return (HD44780_BUSYBIT_MASK & rd) != 0;
}

void
display_instruct(uint16_t instruction)
{
	while (display_isbusy());
	aery_spi_transmit(DISPLAY_SPI, instruction, DISPLAY_SPI_NPCS, true);
}

void
display_wrbyte(uint8_t byte)
{
	aery_spi_transmit(DISPLAY_SPI, 0x200|byte, DISPLAY_SPI_NPCS, true);
}

void
display_putc(char c)
{
	display_wrbyte((uint8_t) c);
}

uint8_t
display_puts(const char *buf)
{
	uint8_t i;
	uint8_t len = strlen(buf);
	for (i = 0; i < len; i++)
		display_putc(buf[i]);
	return i+1;
}

int
display_printf(const char *format, ... )
{
	int n;
	va_list args;

	va_start(args, format);
	n = vsnprintf(dispbuf, MAX_DISPBUF, format, args);
	va_end(args);

	if (n > 0)
		display_puts(dispbuf);
	return n;
}
