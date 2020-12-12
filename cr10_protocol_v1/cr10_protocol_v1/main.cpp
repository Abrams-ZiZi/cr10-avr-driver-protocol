/*
 * Description:		Protocol for a Creality CR10 / Ender 3 board
 * Name:			cr10_protocol_v1.cpp
 * Created:			12.12.2020
 * Author:			Adam Zilizi
 * License:			Open-source 
 * Core:			Atmel ATmega1284p
 * Last update:		12.12.2020
 * Test Desc:		Tested with Windows 10 PC through PuTTY
 * Published on:	https://github.com/Abrams-ZiZi
 */

// IMPORTANT NOTE: this protocol uses the USART.h library created by Ali Gholami (https://atmels.wordpress.com/)
// Please use the included USART.h library, because the original by Ali Gholami has a critical bug which breaks this protocol
// The USART.h library only supports USART0, which is connected to the FT232RL / CH340 USB chip of the CR10 board
// a custom USART library is planned for the future which will support both USART0 (USB) and USART1 (RX and TX pins on EXP1 header)

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "CR10.h"
#include "USART.h"

uint8_t command[10]; // char array to store the received command
uint8_t index = 0; // index for adding received bytes to array
uint8_t receivedByte; // variable to store received byte
uint16_t steps; // 2-byte variable as a parameter for StepperStep()
float resistance; // 4-byte float variable as a parameter for InitTherm()
uint16_t beta; // 2-byte variable as a parameter for InitTherm()
uint8_t endstopState; // variable to store ReadEndstop() return value
uint8_t temperature; // variable to store ThermTemp() return value

float targetTA, targetTB, currentTA, currentTB; // variables for automated heating system
uint8_t enabledA = 0; // auto-heating channel A flag
uint8_t enabledB = 0; // auto-heating channel B flag

// Interrupt handler to manage automated heating (called every 1.05 seconds)
ISR(TIMER3_OVF_vect) {
	// channel A
	if (enabledA) {
		// read current temperature
		currentTA = ThermTemp('A');
		// compare current temperature to target temperature
		if (currentTA < targetTA) {
			PWM1('A', 10); // heat with 10% duty cycle
		}
		else {
			PWM1('A', 0); // turn off heating (0% duty cycle)
		}
	}
	// channel B
	if (enabledB) {
		// read current temperature
		currentTB = ThermTemp('B');
		// compare current temperature to target temperature
		if (currentTB < targetTB) {
			PWM1('B', 10); // heat with 10% duty cycle
		}
		else {
			PWM1('B', 0); // turn off heating (0% duty cycle)
		}
	}
}

void InitTimer3() {
	// set prescaler /256 ==> goes to full every 1.05 seconds
	TCCR3B |= (1 << CS32);
	// enable overflow interrupt
	TIMSK3 |= (1 << TOIE3);
	// enable global interrupts
	sei();
	TCNT3 = 0;
}

int main() {
	// Initialize every component
	init_usart();
	InitPWM0();
	InitPWM1();
	InitEndstop('X');
	InitEndstop('Y');
	InitEndstop('Z');
	
	// Default setup for Stepper motors
	InitStepper('X', 200);
	InitStepper('Y', 200);
	InitStepper('Z', 200);
	InitStepper('E', 200);
	
	// Default setup for Thermistors
	InitTherm('A', 100000, 25, 4900);
	InitTherm('B', 100000, 25, 4900);
	
	// Initialize Timer3 for automated heating system
	InitTimer3();

	while(1) {
		// save received bytes until ':' is received
		do {
			receivedByte = receive();
			command[index] = receivedByte;
			index++;
		} while (receivedByte != ':');
		index = 0; // reset index to 0
		// process command, start with command[1] because command[0] is the start byte '!'
		switch (command[1]) {
			case 'P': // PWM
				switch (command[2]) {
					case '0': // PWM0 - low power connectors (FAN0, FAN1)
						PWM0(command[3]);
						break;
					case '1': // PWM1 - high power connectors (channel A, channel B) (CR10 nozzle, bed)
						PWM1(command[3], command[4]);
						break;
				}
				break;
			case 'S': // Stepper motor
				switch (command[2]) {
					case 'I': // Initialize
						InitStepper(command[3], command[4]);
						break;
					case 'S': // Step command
						// save 2 received bytes into single variable
						steps = ((uint16_t)command[5] << 8) | command[6];
						StepperStep(command[3], command[4], steps);
						break;
				}
				break;
			case 'T': // Thermistor
				switch (command[2]) {
					case 'I': // Initialize
						// convert k0hm to Ohm
						resistance = (float)command[4] * 1000.0;
						// save 2 received bytes into single variable
						beta = ((uint16_t)command[6] << 8) | command[7];
						InitTherm(command[3], resistance, command[5], beta);
						break;
					case 'T': // (read) Temperature
						// read temperature and send the raw binary value (1 byte)
						temperature = (uint8_t)ThermTemp(command[3]);
						send_byte(temperature);
						break;
				}
				break;
			case 'H': // (auto) Heating
				switch (command[2]) {
					case 'E': // Enable
						// channel A
						if (command[3] == 1 || command[3] == 'A') {
							enabledA = 1;
						}
						// channel B
						else if (command[3] == 1 || command[3] == 'B') {
							enabledB = 1;
						}
						break;
					case 'D': // Disable
						// channel A
						if (command[3] == 1 || command[3] == 'A') {
							enabledA = 0;
						}
						// channel B
						else if (command[3] == 1 || command[3] == 'B') {
							enabledB = 0;
						}
						break;
					case 'T': // (set) Temperature
						// channel A
						if (command[3] == 1 || command[3] == 'A') {
							targetTA = command[4];
						}
						// channel B
						else if (command[3] == 1 || command[3] == 'B') {
							targetTB = command[4];
						}
						break;
				}
				break;
			case 'E': // (read) End stop
				// read switch state and send the raw binary value (1 byte)
				endstopState = ReadEndstop(command[2]);
				send_byte(endstopState);
				break;
		}
	}
}
