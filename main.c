#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <BNO055.h>

/*
	Summary of hardware pin usage:

	PC10 -> UART4 TX
	PC11 -> UART4 RX
 */

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


int main (void)
{
	_disable_interrupts();

		uart4_setup();

		setvbuf(stdin,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)
		setvbuf(stdout,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)

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
