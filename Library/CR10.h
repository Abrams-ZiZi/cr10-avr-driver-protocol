/*
 * Description:		Library for interfacing with hardware features of the Creality CR10 / Ender 3 board
 * Name:    		CR10.h
 * Created: 		08.12.2020
 * Author:  		Adam Zilizi
 * License:  		Open-source 
 * Core:    		Atmel ATmega1284p
 * Last update: 	12.12.2020
 * Test Desc:		Tested with standalone test programs and the cr10_protocol_v1 program
 * Published on:	https://github.com/Abrams-ZiZi
 */

#ifndef CR10_H_
#define CR10_H_

#include <avr/io.h>
#include <math.h>

float thermRspecA, thermCspecA, thermBspecA; // variables to store thermistor specs on channel A
float thermRspecB, thermCspecB, thermBspecB; // variables to store thermistor specs on channel B
uint16_t stepsX, stepsY, stepsZ, stepsE; // variables to store stepper motor specs on channels X, Y, Z, E
float stepX, stepY, stepZ, stepE; // variables to store the single microstep (1/16th) angle on channels X, Y, Z, E

// Function to initialize PWM0
void InitPWM0() {
	DDRB |= (1 << PB4); // set OC0B as output
	// enable Compare output mode for OC0B, set mode to Fast PWM
	TCCR0A |= (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	// set prescaler to /1024 (PWM frequency 61Hz)
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCNT0 = 0;
	OCR0B = 0;
}

// user-friendly function alias
void InitFan() {
	return InitPWM0();
}

// Function to set the duty cycle for PWM0
// NOTE: both (fan) connectors are controlled by a single MOSFET, therefore it is not possible to control them separately
void PWM0(uint8_t dutyCycle) {
	OCR0B = dutyCycle * 2.55;
}

// user-friendly function alias
void Fan(uint8_t dutyCycle) {
	return PWM0(dutyCycle);
}

// Function to initialize PWM1
void InitPWM1() {
	DDRD |= (1 << PD4) | (1 << PD5); // set OC1B and OC1A as output
	// enable Compare output mode for OC1A and OC1B, set mode to Fast PWM (TOP == ICR1)
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	// additional flags for Fast PWM (TOP == ICR1) and prescaler /64
	TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
	// set PWM frequency to 7.624Hz (Reprap recommended)
	ICR1 = 32768;
	TCNT1 = 0;
	OCR1A = 0;
	OCR1B = 0;
}

// user-friendly function alias
void InitPowerPWM() {
	return InitPWM1();
}

// Function to set the duty cycle for PWM1
void PWM1(uint8_t channel, uint8_t dutyCycle) {
	// channel A
	if (channel == 1 || channel == 'A') {
		OCR1A = dutyCycle * 327.68;
	}
	// channel B
	else if (channel == 2 || channel == 'B') {
		OCR1B = dutyCycle * 327.68;
	}
}

// user-friendly function alias
void PowerPWM(uint8_t channel, uint8_t dutyCycle) {
	return PWM1(channel, dutyCycle);
}

// Function to initialize the ADC
void InitADC() {
	ADMUX |= (1 << REFS0); // reference voltage AVCC with external cap on AREF
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler /128
	ADCSRA |= (1 << ADEN); // enable ADC
	ADCSRA |= (1 << ADSC); // make one conversion
}

// Function to read from ADC
uint16_t ReadADC(uint8_t channel) {
	ADMUX &= ~(0b00011111); // clear channel selection
	ADMUX |= channel; // set channel
	ADCSRA |= (1 << ADSC); // start conversion
	while (ADCSRA & (1 << ADSC)); // wait for conversion
	return ADCW;
}

// Function to initialize the Thermistors
// Rspec - rated resistance of the thermistor (this is the thermistor's resistance at Cspec)
// Cspec - rated temperature of the thermistor (usually 25 [°C])
// Bspec - Beta value of the thermistor (100k thermistors have their Beta value usually around 4000 - 4500)
void InitTherm(uint8_t channel, float Rspec, float Cspec, float Bspec) {
	InitADC(); // the ADC is initialized
	// channel A (extruder)
	if (channel == 1 || channel ==  'A') {
		DDRA &= ~(1 << PA7); // set ADC7 as input
		// save specs to global variables
		thermRspecA = Rspec;
		thermCspecA = Cspec;
		thermBspecA = Bspec;
	}
	// channel B (bed)
	else if (channel == 2 || channel ==  'B') {
		DDRA &= ~(1 << PA6); // set ADC6 as input
		// save specs to global variables
		thermRspecB = Rspec;
		thermCspecB = Cspec;
		thermBspecB = Bspec;
	}
}

// Function to measure the resistance of the thermistor
float ThermResist(uint8_t channel) {
	uint16_t adcResult;
	// channel A (extruder)
	if (channel == 1 || channel ==  'A') {
		adcResult = ReadADC(7);
	}
	// channel B (bed)
	else if (channel == 2 || channel ==  'B') {
		adcResult = ReadADC(6);
	}
	return 4700.0 * ((float)adcResult / (1023 - adcResult));
}

// Function to calculate the temperature of the thermistor
float ThermTemp(uint8_t channel) {
	// measure thermistor resistance
	float resistance = ThermResist(channel);
	// channel A (extruder)
	if (channel == 1 || channel ==  'A') {
		// use the B-formula to calculate temperature
		return (((thermCspecA + 273.15) * thermBspecA) / ((thermCspecA + 273.15) * log(resistance / thermRspecA) + thermBspecA)) - 273.15;
	}
	// channel B (bed)
	else if (channel == 2 || channel ==  'B') {
		// use the B-formula to calculate temperature
		return (((thermCspecB + 273.15) * thermBspecB) / ((thermCspecB + 273.15) * log(resistance / thermRspecB) + thermBspecB)) - 273.15;
	}
}

// Function to initialize the Stepper motors (A4988 driver)
// steps - number of steps for a full revolution
void InitStepper(uint8_t channel, uint16_t steps) {
	// channel X
	if (channel == 1 || channel ==  'X') {
		// save the number of steps for a full revolution (datasheet spec)
		stepsX = steps;
		// calculate the angle for a single microstep (1/16th step)
		stepX = (360.0 / stepsX) / 16.0;
		// set XYEENABLE and X-STEP as output
		DDRD |= (1 << PD6) | (1 << PD7);
		// set XYEENABLE LOW to enable the A4988 driver
		PORTD &= ~(1 << PD6);
		// set X-DIR as output
		DDRC |= (1 << PC5);
	}
	// channel Y
	else if (channel == 2 || channel == 'Y') {
		// save the number of steps for a full revolution (datasheet spec)
		stepsY = steps;
		// calculate the angle for a single microstep (1/16th step)
		stepY = (360.0 / stepsY) / 16.0;
		// set XYEENABLE as output
		DDRD |= (1 << PD6);
		// set XYEENABLE LOW to enable the A4988 driver
		PORTD &= ~(1 << PD6);
		// set Y-STEP and Y-DIR as output
		DDRC |= (1 << PC6) | (1 << PC7);
	}
	// channel Z
	else if (channel == 3 || channel == 'Z') {
		// save the number of steps for a full revolution (datasheet spec)
		stepsZ = steps;
		// calculate the angle for a single microstep (1/16th step)
		stepZ = (360.0 / stepsZ) / 16.0;
		// set ZENABLE as output
		DDRA |= (1 << PA5);
		// set ZENABLE LOW to enable the A4988 driver
		PORTA &= ~(1 << PA5);
		// set Z-DIR and Z-STEP as output
		DDRB |= (1 << PB2) | (1 << PB3);
	}
	// channel E
	else if (channel == 4 || channel == 'E') {
		// save the number of steps for a full revolution (datasheet spec)
		stepsE = steps;
		// calculate the angle for a single microstep (1/16th step)
		stepE = (360.0 / stepsE) / 16.0;
		// set XYEENABLE as output
		DDRD |= (1 << PD6);
		// set XYEENABLE LOW to enable the A4988 driver
		PORTD &= ~(1 << PD6);
		// set E-DIR and E-STEP as output
		DDRB |= (1 << PB0) | (1 << PB1);
	}
}

// Function to rotate the stepper motors
// direction - 'C' - clock-wise; 'A' - anti-clockwise
// angle - angle in degrees
void StepperAngle(uint8_t channel, uint8_t direction, float angle) {
	// variable to store the number of microsteps needed
	float steps;
	// channel X
	if (channel == 1 || channel ==  'X') {
		// calculate number of microsteps
		steps = angle / stepX;
		if (direction == 'C') {
			// set X-DIR HIGH for clockwise
			PORTC |= (1 << PC5);
		}
		else if (direction == 'A') {
			// set X-DIR LOW for anti-clockwise
			PORTC &= ~(1 << PC5);
		}
		// send signals out of X-STEP for microstepping
		for (int i = 0; i < steps; i++) {
			PORTD |= (1 << PD7);
			_delay_us(50);
			PORTD &= ~(1 << PD7);
			_delay_us(50);
		}
	}
	// channel Y
	else if (channel == 2 || channel == 'Y') {
		// calculate number of microsteps
		steps = angle / stepY;
		if (direction == 'C') {
			// set Y-DIR HIGH for clockwise
			PORTC |= (1 << PC7);
		}
		else if (direction == 'A') {
			// set Y-DIR LOW for anti-clockwise
			PORTC &= ~(1 << PC7);
		}
		// send signals out of Y-STEP for microstepping
		for (int i = 0; i < steps; i++) {
			PORTC |= (1 << PC6);
			_delay_us(50);
			PORTC &= ~(1 << PC6);
			_delay_us(50);
		}
	}
	// channel Z
	else if (channel == 3 || channel == 'Z') {
		// calculate number of microsteps
		steps = angle / stepZ;
		if (direction == 'C') {
			// set Z-DIR HIGH for clockwise
			PORTB |= (1 << PB2);
		}
		else if (direction == 'A') {
			// set Z-DIR LOW for anti-clockwise
			PORTB &= ~(1 << PB2);
		}
		// send signals out of Z-STEP for microstepping
		for (int i = 0; i < steps; i++) {
			PORTB |= (1 << PB3);
			_delay_us(50);
			PORTB &= ~(1 << PB3);
			_delay_us(50);
		}
	}
	// channel E
	else if (channel == 4 || channel == 'E') {
		// calculate number of microsteps
		steps = angle / stepE;
		if (direction == 'C') {
			// set E-DIR HIGH for clockwise
			PORTB |= (1 << PB0);
		}
		else if (direction == 'A') {
			// set E-DIR LOW for anti-clockwise
			PORTB &= ~(1 << PB0);
		}
		// send signals out of E-STEP for microstepping
		for (int i = 0; i < steps; i++) {
			PORTB |= (1 << PB1);
			_delay_us(50);
			PORTB &= ~(1 << PB1);
			_delay_us(50);
		}
	}
}

// Function to rotate the stepper motors
// direction - 'C' - clock-wise; 'A' - anti-clockwise
// microsteps - number of microsteps (1/16th) to rotate (to calculate back to an angle (in degrees) just divide the single-step datasheet spec with 16)
void StepperStep(uint8_t channel, uint8_t direction, uint32_t microsteps) {
	// channel X
	if (channel == 1 || channel ==  'X') {
		if (direction == 'C') {
			// set X-DIR HIGH for clockwise
			PORTC |= (1 << PC5);
		}
		else if (direction == 'A') {
			// set X-DIR LOW for anti-clockwise
			PORTC &= ~(1 << PC5);
		}
		// send signals out of X-STEP for microstepping
		for (int i = 0; i < microsteps; i++) {
			PORTD |= (1 << PD7);
			_delay_us(50);
			PORTD &= ~(1 << PD7);
			_delay_us(50);
		}
	}
	// channel Y
	else if (channel == 2 || channel == 'Y') {
		if (direction == 'C') {
			// set Y-DIR HIGH for clockwise
			PORTC |= (1 << PC7);
		}
		else if (direction == 'A') {
			// set Y-DIR LOW for anti-clockwise
			PORTC &= ~(1 << PC7);
		}
		// send signals out of Y-STEP for microstepping
		for (int i = 0; i < microsteps; i++) {
			PORTC |= (1 << PC6);
			_delay_us(50);
			PORTC &= ~(1 << PC6);
			_delay_us(50);
		}
	}
	// channel Z
	else if (channel == 3 || channel == 'Z') {
		if (direction == 'C') {
			// set Z-DIR HIGH for clockwise
			PORTB |= (1 << PB2);
		}
		else if (direction == 'A') {
			// set Z-DIR LOW for anti-clockwise
			PORTB &= ~(1 << PB2);
		}
		// send signals out of Z-STEP for microstepping
		for (int i = 0; i < microsteps; i++) {
			PORTB |= (1 << PB3);
			_delay_us(50);
			PORTB &= ~(1 << PB3);
			_delay_us(50);
		}
	}
	// channel E
	else if (channel == 4 || channel == 'E') {
		if (direction == 'C') {
			// set E-DIR HIGH for clockwise
			PORTB |= (1 << PB0);
		}
		else if (direction == 'A') {
			// set E-DIR LOW for anti-clockwise
			PORTB &= ~(1 << PB0);
		}
		// send signals out of E-STEP for microstepping
		for (int i = 0; i < microsteps; i++) {
			PORTB |= (1 << PB1);
			_delay_us(50);
			PORTB &= ~(1 << PB1);
			_delay_us(50);
		}
	}
}

// Function to initialize End stops (practically buttons / switches)
void InitEndstop(uint8_t channel) {
	// channel X
	if (channel == 1 || channel == 'X') {
		// set X-STOP as input
		DDRC &= ~(1 << PC2);
	}
	// channel Y
	else if (channel == 2 || channel == 'Y') {
		// set Y-STOP as input
		DDRC &= ~(1 << PC3);
	}
	// channel Z
	else if (channel == 3 || channel == 'Z') {
		// set Z-STOP as input
		DDRC &= ~(1 << PC4);
	}
}

// user-friendly function alias
void InitButton(uint8_t channel) {
	return InitEndstop(channel);
}

// Function to read the End stop state
uint8_t ReadEndstop(uint8_t channel) {
	// channel X
	if (channel == 1 || channel == 'X') {
		return PINC & (1 << PC2);
	}
	// channel Y
	else if (channel == 2 || channel == 'Y') {
		return PINC & (1 << PC3);
	}
	// channel Z
	else if (channel == 3 || channel == 'Z') {
		return PINC & (1 << PC4);
	}
}

// user-friendly function alias
uint8_t ReadButton(uint8_t channel) {
	return ReadEndstop(channel);
}

// Function to check whether an End stop has been triggered since the last function call
uint8_t TriggeredEndstop(uint8_t channel) {
	// static variables to store End stop states
	static uint8_t previousPressedX = 0;
	static uint8_t currentPressedX = 0;
	static uint8_t previousPressedY = 0;
	static uint8_t currentPressedY = 0;
	static uint8_t previousPressedZ = 0;
	static uint8_t currentPressedZ = 0;
	// channel X
	if (channel == 1 || channel == 'X') {
		// save previous button state
		previousPressedX = currentPressedX;
		// read new button state
		if ((PINC & (1 << PC2)) == 0) {
			currentPressedX = 1;
		}
		else {
			currentPressedX = 0;
		}
		// check if the button transitioned from not being pressed to being pressed
		if (currentPressedX == 1 && previousPressedX == 0) {
			return 1;
		}
		else {
			return 0;
		}
	}
	// channel Y
	else if (channel == 2 || channel == 'Y') {
		// save previous button state
		previousPressedY = currentPressedY;
		// read new button state
		if ((PINC & (1 << PC3)) == 0) {
			currentPressedY = 1;
		}
		else {
			currentPressedY = 0;
		}
		// check if the button transitioned from not being pressed to being pressed
		if (currentPressedY == 1 && previousPressedY == 0) {
			return 1;
		}
		else {
			return 0;
		}
	}
	// channel Z
	else if (channel == 3 || channel == 'Z') {
		// save previous button state
		previousPressedZ = currentPressedZ;
		// read new button state
		if ((PINC & (1 << PC4)) == 0) {
			currentPressedZ = 1;
		}
		else {
			currentPressedZ = 0;
		}
		// check if the button transitioned from not being pressed to being pressed
		if (currentPressedZ == 1 && previousPressedZ == 0) {
			return 1;
		}
		else {
			return 0;
		}
	}
}

// user-friendly function alias
uint8_t TriggeredButton(uint8_t channel) {
	return TriggeredEndstop(channel);
}

#endif /* CR10_H_ */
