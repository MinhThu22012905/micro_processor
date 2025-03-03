#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h> // Required for strcat

#define Tang PINB0
#define Giam PINB1
#define Dao PINB2
#define Stop PINB3
#define LCD_RS PA0
#define LCD_EN PA2
#define LCD_RW PA1
#define LCD_D4 PA4
#define LCD_D5 PA5
#define LCD_D6 PA6
#define LCD_D7 PA7
#define LCD_DATA PORTA
#define LCD_CTRL PORTA
#define LCD_DATA_DDR DDRA
#define LCD_CTRL_DDR DDRA

volatile unsigned long encoder = 0;
volatile unsigned int speed = 0;
volatile unsigned int temp = 0;
volatile unsigned char direction = 0;

void lcd_command(unsigned char cmd) {
	LCD_DATA = (LCD_DATA & 0x0F) | (cmd & 0xF0);
	LCD_CTRL &= ~(1 << LCD_RS);
	LCD_CTRL &= ~(1 << LCD_RW);
	LCD_CTRL |= (1 << LCD_EN);
	_delay_us(1);
	LCD_CTRL &= ~(1 << LCD_EN);
	_delay_us(200);

	LCD_DATA = (LCD_DATA & 0x0F) | (cmd << 4);
	LCD_CTRL |= (1 << LCD_EN);
	_delay_us(1);
	LCD_CTRL &= ~(1 << LCD_EN);
	_delay_ms(2);
}

void lcd_data(unsigned char data) {
	LCD_DATA = (LCD_DATA & 0x0F) | (data & 0xF0);
	LCD_CTRL |= (1 << LCD_RS);
	LCD_CTRL &= ~(1 << LCD_RW);
	LCD_CTRL |= (1 << LCD_EN);
	_delay_us(1);
	LCD_CTRL &= ~(1 << LCD_EN);
	_delay_us(200);

	LCD_DATA = (LCD_DATA & 0x0F) | (data << 4);
	LCD_CTRL |= (1 << LCD_EN);
	_delay_us(1);
	LCD_CTRL &= ~(1 << LCD_EN);
	_delay_ms(2);
}

void lcd_init() {
	LCD_DATA_DDR = 0xF0;
	LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_EN);
	_delay_ms(20);

	lcd_command(0x02);
	lcd_command(0x28);
	lcd_command(0x0C);
	lcd_command(0x06);
	lcd_command(0x01);
}

void lcd_set_cursor(unsigned char row, unsigned char col) {
	unsigned char pos = (row == 0) ? col | 0x80 : col | 0xC0;
	lcd_command(pos);
}

void lcd_print(char *str) {
	while (*str) {
		lcd_data(*str++);
	}
}

void lcd_display_info() {
	lcd_set_cursor(0, 0);
	lcd_print("PWM: ");
	char buffer[20];  // Buffer for PWM value
	itoa(temp, buffer, 10);  // Convert PWM value to string
	lcd_print(buffer);

	lcd_set_cursor(1, 0);
	lcd_print("Speed: ");

	// Calculate speed (rpm), assuming temp represents the PWM signal
	float speed_rpm = (float)temp / 7999.0 * 60;  // Example calculation
	char speed_buffer[20];  // Buffer for speed value

	// Convert speed to string with 2 decimal places
	dtostrf(speed_rpm, 5, 2, speed_buffer);

	// Append " rpm" to the speed value
	strcat(speed_buffer, " rpm");

	// Print the resulting string
	lcd_print(speed_buffer);
}

void pwm_init() {
	DDRD |= (1 << PIND5);
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	ICR1 = 7999;
	OCR1A = 0;
}

ISR(TIMER2_OVF_vect) {
	uint8_t pinb_state = PINB;

	// Increase PWM duty cycle when "Tang" button is pressed
	if (!(pinb_state & (1 << Tang))) {
		temp = (temp < 7999) ? temp + 500 : 7999; 
		// Limit max PWM value to 7999
	}
	// Decrease PWM duty cycle when "Giam" button is pressed
	if (!(pinb_state & (1 << Giam))) {
		temp = (temp > 500) ? temp - 500 : 0;  // Limit min PWM value to 0
	}

	// Stop motor when "Stop" button is pressed
	if (!(pinb_state & (1 << Stop))) {
		temp = 0;       // Set PWM duty cycle to 0
		OCR1A = 0;      // Immediately apply 0 duty cycle
		TCNT1 = 0;      // Reset Timer1 counter
		PORTD &= ~((1 << PORTD3) | (1 << PORTD5)); // Clear both direction pins (stop motor completely)
	}

	// Reverse motor direction when "Dao" button is pressed
	if (!(pinb_state & (1 << Dao))) {
		OCR1A = 0; // Temporarily stop PWM to safely reverse direction
		_delay_ms(50);  // Allow the motor to stop completely

		// Toggle motor direction
		direction = !direction;

		if (direction) {
			PORTD |= (1 << PORTD3);  // Forward direction
			PORTD &= ~(1 << PORTD5);
			} else {
			PORTD |= (1 << PORTD5);  // Reverse direction
			PORTD &= ~(1 << PORTD3);
		}

		// Resume PWM after direction change
		OCR1A = temp;  // Restore the PWM duty cycle
	}
}

int main(void) {
	DDRB = 0x00;
	PORTB = 0x0F;
	DDRA = 0xF0;
	DDRD = 0x2A;

	pwm_init();
	lcd_init();

	PORTD |= (1 << PIND1); // Initial direction set

	TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);
	TIMSK = (1 << TOIE2);

	sei(); // Enable global interrupts

	while (1) {
		OCR1A = temp; // Update PWM duty cycle continuously
		lcd_display_info();
		_delay_ms(100);
	}
}
