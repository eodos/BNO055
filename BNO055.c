#include <BNO055.h>

static volatile enum BNO055_states state;
static volatile enum BNO055_operations operation;
static volatile uint8_t packet_buffer[MAX_BYTES];

void BNO055_write_single_register(uint8_t register_address, uint8_t data)
{
	operation = WRITE_REGISTER;
	state = RECEIVING_HEADER;

	usart_send_blocking(BNO055_UART_PORT, 0xAA);
	usart_send_blocking(BNO055_UART_PORT, 0x00);
	usart_send_blocking(BNO055_UART_PORT, register_address);
	usart_send_blocking(BNO055_UART_PORT, 0x01);
	usart_send_blocking(BNO055_UART_PORT, data);
}

void BNO055_write_register(uint8_t register_address, uint8_t length, uint8_t *data)
{
	operation = WRITE_REGISTER;
	state = RECEIVING_HEADER;

	usart_send_blocking(BNO055_UART_PORT, 0xAA);
	usart_send_blocking(BNO055_UART_PORT, 0x00);
	usart_send_blocking(BNO055_UART_PORT, register_address);
	usart_send_blocking(BNO055_UART_PORT, 0x01);
	for (int i=0; i<length; i++)
	{
		usart_send_blocking(BNO055_UART_PORT, data[i]);
	}
}


void BNO055_read_single_register(uint8_t register_address)
{
	operation = READ_REGISTER;
	state = RECEIVING_HEADER;

	usart_send_blocking(BNO055_UART_PORT, 0xAA);
	usart_send_blocking(BNO055_UART_PORT, 0x01);
	usart_send_blocking(BNO055_UART_PORT, register_address);
	usart_send_blocking(BNO055_UART_PORT, 0x01);
}

void BNO055_read_register(uint8_t register_address, uint8_t length)
{
	operation = READ_REGISTER;
	state = RECEIVING_HEADER;

	usart_send_blocking(BNO055_UART_PORT, 0xAA);
	usart_send_blocking(BNO055_UART_PORT, 0x01);
	usart_send_blocking(BNO055_UART_PORT, register_address);
	usart_send_blocking(BNO055_UART_PORT, length);
}

void BNO055_interrupt_handler(uint8_t byte)
{
	static uint8_t packet_length;
	static uint8_t i = 0;

	switch (operation)
	{
		case READ_REGISTER:
			switch (state)
			{
				case RECEIVING_HEADER:
					if (byte == ACK_READ_SUCCESS)
					{
						state = RECEIVING_LENGTH;
					}
					else
					{
						state = ERROR;
					}
					break;
				case RECEIVING_LENGTH:
					packet_length = byte;
					i = 0;
					state = RECEIVING_DATA;
					break;
				case RECEIVING_DATA:
					packet_buffer[i] = byte;
					i++;
					packet_length--;
					if (packet_length == 0)
					{
						state = FINISHED;
					}
					break;
				case ERROR:
				default:
					break;
			}
			break;
		case WRITE_REGISTER:
			switch(state)
			{
				case RECEIVING_HEADER:
					if (byte == ACK_WRITE)
					{
						state = RECEIVING_OPERATION_RESULT;
					}
					else
					{
						state = ERROR;
					}
					break;
				case RECEIVING_OPERATION_RESULT:
					if (byte == ACK_WRITE_SUCCESS)
					{
						state = FINISHED;
					}
					else
					{
						state = ERROR;
					}
				case ERROR:
				default:
					break;
			}
			break;
	}
}

void BNO055_get_data(void)
{
	BNO055_read_register(0x14, 20);
}

imu_data BNO055_process_buffer(void)
{
	union {
		uint8_t input[2];
		int16_t output;
	} bytes_to_signed_int16;

	imu_data data;

		/* Gyroscope x */
	bytes_to_signed_int16.input[0] = packet_buffer[0];
	bytes_to_signed_int16.input[1] = packet_buffer[1];

	data.gyro_x = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Gyroscope y */
	bytes_to_signed_int16.input[0] = packet_buffer[2];
	bytes_to_signed_int16.input[1] = packet_buffer[3];

	data.gyro_y = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Gyroscope z */
	bytes_to_signed_int16.input[0] = packet_buffer[4];
	bytes_to_signed_int16.input[1] = packet_buffer[5];

	data.gyro_z = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Heading */
	bytes_to_signed_int16.input[0] = packet_buffer[6];
	bytes_to_signed_int16.input[1] = packet_buffer[7];

	data.heading = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Roll */
	bytes_to_signed_int16.input[0] = packet_buffer[8];
	bytes_to_signed_int16.input[1] = packet_buffer[9];

	data.roll = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Pitch */
	bytes_to_signed_int16.input[0] = packet_buffer[10];
	bytes_to_signed_int16.input[1] = packet_buffer[11];

	data.pitch = (float)bytes_to_signed_int16.output/(float)16.0f;

	/* Quaternion w */
	bytes_to_signed_int16.input[0] = packet_buffer[6];
	bytes_to_signed_int16.input[1] = packet_buffer[7];

	data.quaternion_w = (float)bytes_to_signed_int16.output/(float)16384.0f;

	/* Quaternion x */
	bytes_to_signed_int16.input[0] = packet_buffer[8];
	bytes_to_signed_int16.input[1] = packet_buffer[9];

	data.quaternion_x = (float)bytes_to_signed_int16.output/(float)16384.0f;

	/* Quaternion y */
	bytes_to_signed_int16.input[0] = packet_buffer[10];
	bytes_to_signed_int16.input[1] = packet_buffer[11];

	data.quaternion_y = (float)bytes_to_signed_int16.output/(float)16384.0f;

	/* Quaternion z */
	bytes_to_signed_int16.input[0] = packet_buffer[10];
	bytes_to_signed_int16.input[1] = packet_buffer[11];

	data.quaternion_z = (float)bytes_to_signed_int16.output/(float)16384.0f;

	return data;
}