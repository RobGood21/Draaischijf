/*
 Name:		Draaischijf.ino
 Created:	12/29/2018 8:26:39 PM
 Author:	Rob Antonisse
*/

unsigned long time;
unsigned long periode;
byte COM_reg; 
byte MOT_mode;
byte MOT_speed=5;
byte MOT_speedcount=0;
byte fase;
byte portc;



void setup() {
	Serial.begin(9600);
	DDRB = 0xFF;
	DDRC = 0x00; //port C as input
	PORTC = 0xFF; //pull up resistors to port C
}

// the loop function runs over and over again until power down or reset
void ST_clear() {
	PORTB = PORTB >> 4;
	PORTB = PORTB << 4;
}
void ST_step() {
	MOT_speedcount++;
	if (MOT_speedcount > MOT_speed) {
		MOT_speedcount = 0;
		Serial.println(fase);
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
}
void SW_cl() {

	byte changed;
	changed = (PINC^portc);

	for (byte i = 0; i < 8; i++) {
		if (bitRead(changed, i) == true & bitRead(PINC,i)==false) {
			//PINB |= (1 << i); //toggle 
			switch(i) {
			case 0:	
				MOT_mode = 1;
				break;
			case 1:
				MOT_mode = 0;
				ST_clear();
				break;
			case 2:
				MOT_mode = 2;
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:
				break;
			case 6:
				break;
			case 7:
				break;
			}
		}
	}
	portc = PINC;
}



void loop() {

	if (millis() - time > 5) {
		time = millis();
		SW_cl();
		if(MOT_mode !=0) ST_step();
 }
	   
}
