/*
 Name:		Draaischijf.ino
 Created:	12/29/2018 8:26:39 PM
 Author:	Rob Antonisse
*/

unsigned long SW_time;
//unsigned long SW_periode;
byte COM_reg;
byte MOT_mode;
unsigned long MOT_time;
byte POS_last; //last posion on place
byte POS_cur; // current position
byte POS_rq; //requested position
byte fase;
byte portc;



void setup() {
	//Serial.begin(9600);
	DDRB = 0xFF;
	DDRC = 0x00; //port C as input
	PORTC |= (1 << 0); PORTC |= (1 << 1); PORTC |= (1 << 2); //pins A0 -A2 set pullup resister
}


void ST_clear() {
	PORTB = PORTB >> 4;
	PORTB = PORTB << 4;
}
void ST_step() {

	//Serial.println(fase);
	ST_clear();
	//PINB |= ((1 << 5));
	switch (fase) {
	case 0:
		PORTB |= (1 << 0);
		break;
	case 1:
		PORTB |= (1 << 0);
		PORTB |= (1 << 1);
		break;
	case 2:
		PORTB |= (1 << 1);
		break;
	case 3:
		PORTB |= (1 << 1);
		PORTB |= (1 << 2);
		break;
	case 4:
		PORTB |= (1 << 2);
		break;
	case 5:
		PORTB |= (1 << 2);
		PORTB |= (1 << 3);
		break;
	case 6:
		PORTB |= (1 << 3);
		break;
	case 7:
		PORTB |= (1 << 3);
		PORTB |= (1 << 0);
		break;
	}

	if (MOT_mode == 1) {
		fase++;
		if (fase > 7) fase = 0;
	}
	else { //mot mode=2
		fase--;
		if (fase > 7)fase = 7;

	}
}
void SW_cl() {

	byte changed;
	byte pos;
	changed = (PINC^portc);
	if (changed > 0) {
		//Serial.println(PINC);
		for (byte i = 0; i < 3; i++) { //read three switches
			if (bitRead(changed, i) == true & bitRead(PINC, i) == false) {
				switch (i) {
				case 0:
					//schijf linksom (rood)
					MOT_mode = 1;
					POS_rq = POS_last - 1;
					if (POS_rq < 1 | POS_rq > 7)POS_rq = 7;
					if (POS_rq == 6)POS_rq = 5;
					break;
				case 1:
					//schijf stop
					ST_stop();
					
					break;
				case 2:
					//schijf rechtom (zwart)
					MOT_mode = 2;
					POS_rq = POS_last + 1;
					if (POS_rq == 6)POS_rq = 7;
					if (POS_rq > 7)POS_rq = 1;
					break;
				}
				Serial.print("request: ");
				Serial.println(POS_rq);
			}
		}
		changed = changed >> 3;
		if (changed > 0) {
			POS_cur = PINC >> 3;
			if (POS_cur > 0)POS_last = POS_cur;
			ST_position();
		}
	}
	portc = PINC;
}
void ST_stop() {
MOT_mode = 0;
ST_clear();
}
void ST_position() {
	Serial.println(POS_cur);
	if (MOT_mode > 0) {
		if (POS_cur > 0) {
			if (POS_cur == POS_rq) ST_stop();
		}
	}
	else {
		//positie verandert zonder dat de schrijf draait, hier een foutafhandeling maken. 
		Serial.println("positie veranderd zonder beweging");
	}
}

void loop() {


	if (millis() - MOT_time > 5) {
		MOT_time = millis();
		if (MOT_mode != 0) ST_step();
		}
	

	if (millis() - SW_time > 20) {
		SW_time = millis();
		SW_cl();		
	}
}
