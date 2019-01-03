/*
 Name:		WMServo.ino
 Created:	1/3/2019 9:13:07 PM
 Author:	Rob Antonisse
 Website:	www.wisselmotor.nl

 Program to control a servo from Arduino (for start only one...)
 Servo on pin 7

*/
byte SERVO_reg = 0;

unsigned long SERVO_mut;
unsigned long SERVO_time;
unsigned int SERVO_pos;
unsigned long SERVO_clock;
unsigned int SERVO_min;
unsigned int SERVO_max;

byte count = 0;

void setup() {
	DDRD |= (1 << 7); //set PIN7 as output
	SERVO_pos = 1900;
	SERVO_min = 1100;
	SERVO_max = 1900;
}

void SERVO_speed() {
	if (millis() - SERVO_mut > 1) {
		SERVO_mut = millis();
		if (bitRead(SERVO_reg, 1) == true) {
			SERVO_pos++;
			if (SERVO_pos == SERVO_max) {
				SERVO_reg &= ~(1 << 1);
				count++;
			}

		}
		else {
			SERVO_pos--;
			if (SERVO_pos == SERVO_min)SERVO_reg |= (1 << 1);
		}
	}
}

void SERVO_run() {
	SERVO_speed();
	if (bitRead(SERVO_reg, 0) == false) {
		if (millis() - SERVO_clock > 20) { //runs on 50HZ
			SERVO_clock = millis();
			SERVO_reg |= (1 << 0);
			SERVO_time = micros();
			//set pin7 high PORTD 7
			PORTD |= (1 << 7);

		}
	}
	else {
		if (micros() - SERVO_time > SERVO_pos) {
			PORTD &= ~(1 << 7); // set pin 7 low
			SERVO_reg &= ~(1 << 0);
		}
	}
}


void loop() {
	if (count < 10)SERVO_run();
}
