<<<<<<< HEAD
=======
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
	C Standard Library/Newlib includes:
 */

>>>>>>> 2c4b2916bac3d21373c9352173ab883933fbb8ac
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

<<<<<<< HEAD
#include <BNO055.h>

/*
	Summary of hardware pin usage:

	PC10 -> UART4 TX
	PC11 -> UART4 RX
 */
=======
/*
	HAL/HW-specific includes:
 */

#include <hal_common_includes.h>
#include <robot_config.h>

/*
	Avionics software-specific includes:
 */

#include <interrupts.h>
#include <mission_timekeeper.h>
#include <imu.h>
#include <QuadRotor_PWM.h>
#include <rpi_comms.h>

#include <BNO055.h>

<<<<<<< HEAD
>>>>>>> 2c4b2916bac3d21373c9352173ab883933fbb8ac
=======
/*
	Shamelessly stolen from I2C example in libopencm3-examples,
	but sharing is caring, right? Right? Okay.
 */

// #define LRED GPIOE, GPIO9
// #define LORANGE GPIOE, GPIO10
// #define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
// #define LRED2 GPIOE, GPIO13
// #define LORANGE2 GPIOE, GPIO14
// #define LGREEN2 GPIOE, GPIO15

#define BTN_A  0
#define BTN_B  1
#define BTN_X  2
#define BTN_Y  3
#define LT_BTN 4
#define RT_BTN 5 // Doesn't work for some reason, maybe busted controller??
#define VIEW_BTN  6
#define MENU_BTN  7
#define LEFT_STICK_BUTTON   8
#define RIGHT_STICK_BUTTON  9

// #define INCLUDE_PWM_TEST_SHELL	1
// #define ENABLE_PWM_TEST_SHELL	1

/*
	Summary of hardware pin usage:

	PA9,10 			->			USART1, DEBUG
	PA2,3 			-> 			USART2, Raspberry Pi
	PE9,11,13,14 	-> 			Timer1 PWM Output
	PC6,7,PD6,7 	-> 			Timer3 PWM Output
	PD2 			-> 			Servo Power enable
 */

/*
 A basic routine to adjust system clock settings to get SYSCLK
 to 64 MHz, and have AHB buses at 64 MHz, and APB Bus at 32 MHz (its max speed)
 Copied from:
 https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f3/stm32f3-discovery/adc/adc.c
 */

/*
	External oscillator required to clock PLL at 72 MHz:
 */
static void set_system_clock(void)
{
	rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
	// rcc_clock_setup_hsi(&rcc_hsi_8mhz[RCC_CLOCK_64MHZ]);
}

static void setup_led_pins(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9 | GPIO10 | GPIO11);
	gpio_set(GPIOE, GPIO8 | GPIO9 | GPIO10 | GPIO11);
}

static void usart1_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	// Unmask receive interrupt
	usart_enable_rx_interrupt(USART1);

	// Make sure the interrupt is routed through the NVIC
	nvic_enable_irq(NVIC_USART1_EXTI25_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void usart2_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO2 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	// Unmask receive interrupt
	usart_enable_rx_interrupt(USART2);

	// Make sure the interrupt is routed through the NVIC
	nvic_enable_irq(NVIC_USART2_EXTI26_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART2);
}
>>>>>>> parent of 2c4b291... Update main.c

static void uart4_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_UART4);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
	gpio_set_af(GPIOC, GPIO_AF5, GPIO10 | GPIO11);

	/* Setup UART parameters. */
	usart_set_baudrate(UART4, 115200);//921600
	usart_set_databits(UART4, 8);
	usart_set_stopbits(UART4, USART_STOPBITS_1);
	usart_set_mode(UART4, USART_MODE_TX_RX);
	usart_set_parity(UART4, USART_PARITY_NONE);
	usart_set_flow_control(UART4, USART_FLOWCONTROL_NONE);

	// Unmask receive interrupt
	usart_enable_rx_interrupt(UART4);
	// Make sure the interrupt is routed through the NVIC
	nvic_enable_irq(NVIC_UART4_EXTI34_IRQ);

	/* Finally enable the USART. */
	usart_enable(UART4);
}

static void usart_setup(void)
{
	usart1_setup();
	usart2_setup();
	uart4_setup();
}

static void init_user_button(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO15);
}

static void init_adc_input(void)
{
	//ADC
	rcc_periph_clock_enable(RCC_ADC34);
	rcc_periph_clock_enable(RCC_GPIOB);
	//ADC
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	adc_off(ADC3);
	RCC_CFGR2 |= (uint32_t)0x10003400;
	adc_set_clk_prescale(ADC_CCR_CKMODE_DIV4);
	adc_set_single_conversion_mode(ADC3);
	adc_disable_external_trigger_regular(ADC3);
	adc_set_right_aligned(ADC3);

	adc_set_sample_time_on_all_channels(ADC3, ADC_SMPR1_SMP_1DOT5CYC);
	adc_set_resolution(ADC3, ADC_CFGR_RES_12_BIT);
	adc_power_on(ADC3);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)
		__asm__("nop");

	uint8_t channel_array[16];
	int j = 0;
	for(j=0; j<16; ++j)
	{
		channel_array[j]=1;
	}

	adc_set_regular_sequence(ADC3, 1, channel_array);
}

int main (void)
{
	_disable_interrupts();

		set_system_clock();
		setup_led_pins();
		systick_setup();
		usart_setup();
		init_user_button();

		setvbuf(stdin,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)
		setvbuf(stdout,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)

		init_mission_timekeeper();
		init_pi_comms();

	_enable_interrupts();

	timekeeper_delay(500U);

	volatile imu_data data;

	data.heading = 0;
	data.roll = 0;
	data.pitch = 0;

	data.gyro_x = 0;
	data.gyro_y = 0;
	data.gyro_z = 0;

	data.quaternion_w = 0;
	data.quaternion_x = 0;
	data.quaternion_y = 0;
	data.quaternion_z = 0;

	BNO055_get_data();

	timekeeper_delay(500);

	data = BNO055_process_buffer();

	while(1);

	while (1)
	{
		gpio_toggle(GPIOE, GPIO11);
		timekeeper_delay(500);
	}

	return 0;
}
