/*
 Name:		TreinLift.ino
 Created:	9/22/2020 10:13:11 AM
 Author:	rob Antonisse

 Design for train vertical fiddle yard. Treinlift
 Steppermotor 23HS5628-08 stepper driver TB6600

*/

unsigned long slowtime;
unsigned long speedtime;
byte speed=100;
byte switchstatus = 0xFF;


void setup() {
	Serial.begin(9600);

	//ports
	PORTC |= (15 << 0); //pullup  A0~A3
	DDRB |= (14 << 0); //Pin9~Pin11 as outputs

	//timer 2 settings on pin11 compare match
	TCCR2A |= (1 << 6); //toggle pin6
	TCCR2A |= (1 << 1); 
	TCCR2B |= (1 << 3);
	TCCR2B |= (1 << 1); //prescaler max
	OCR2A = 255; //snelheid
	//TIMSK2 |= (1 << 1); //enable timer 
}

void SW_read() { //lezen van schakelaars
	byte sr; byte changed;
	sr = PINC;
	sr |= (B11110000 << 0); //set bits 7~4
	changed = sr ^ switchstatus;
	for (byte i = 0; i < 4; i++) {
		//Serial.println("hier");
		if (changed & (1 << i) & ~sr & (1 << i)) SW_exe(i);
	}
	switchstatus = sr;
}
void SW_exe(byte sw) {
	Serial.println(sw);
	switch (sw) {
	case 0:
		
		break;
	case 1:
		
		break;
	case 2:
		PORTB ^= (1 << 2); //direction
		break;
	case 3:
		PORTB ^= (1 << 1); //enabled
		break;
	}
}
void loop() {
	if (millis() - slowtime > 100) {
		slowtime = millis();
		SW_read();
	}

	//if (millis() - speedtime > speed) {
	//	speedtime = millis();
	//	PORTB ^= (1 << 3);
	//}
}
