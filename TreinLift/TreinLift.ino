/*
 Name:		TreinLift.ino
 Created:	9/22/2020 10:13:11 AM
 Author:	rob Antonisse

 Design for train vertical fiddle yard. Treinlift
 Steppermotor 23HS5628-08 stepper driver TB6600

*/


#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <splash.h>
#include <Adafruit_SSD1306.h>

#define cd display.clearDisplay()
#define regel1 DSP_settxt(0, 2, 2) //parameter eerste regel groot
#define regel2 DSP_settxt(0, 23, 2) //parameter tweede regel groot
#define regel3 DSP_settxt(10,23,2) //value tweede regel groot
#define regel1s DSP_settxt(0, 2, 1) //value eerste regel klein
#define regel2s DSP_settxt(0, 0, 1) //X Y size X=0 Y=0 geen cursor verplaatsing
#define regelb DSP_settxt(30,15,6)

#define up PORTB |=(1<<2);GPIOR0 |=(1<<1);
#define down PORTB &=~(1<<2);GPIOR0 &=~(1<<1);
#define speed OCR2A
#define start TCCR2B |= (1 << 3); TIMSK2 |= (1 << 1); //enable interupt
#define stop TCCR2B &=~(1 << 3);  TIMSK2 &=~(1 << 1); //disable interupt

Adafruit_SSD1306 display(128, 64, &Wire, -1);
unsigned long slowtime;
unsigned long speedtime;
//unsigned long wait;
volatile unsigned long POS;
volatile unsigned long POS_rq;
volatile unsigned long etage[8];
byte etage_status; //bit0 false positie bepaald, enz lezen eeprom 100
byte etage_rq;


//byte speed = 100;
byte switchstatus[3];
byte switchcount;
byte PRG_fase;
byte PRG_level;
void setup() {
	Serial.begin(9600);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	cd; //clear display
	regel1s; display.print("www.wisselmotor.nl");
	regel2; display.print("Treinlift");
	display.display();
	delay(500);

	//ports
	PORTC |= (15 << 0); //pullup  A0~A3
	DDRB |= (14 << 0); //Pin9~Pin11 as outputs
	//DDRD |= (B11100000 << 0); //pin 7,6,5 output

	//factory reset
	DDRD |= (1 << 7);
	//Serial.println(PINC);
	if (PINC == 54) {
		FACTORY();
		delay(500);
	}

	//timer 2 settings on pin11 compare match
	TCCR2A |= (1 << 6); //toggle pin6
	TCCR2A |= (1 << 1);
	//TCCR2B |= (1 << 3);
	TCCR2B |= (1 << 1); //prescaler max
	//OCR2A = ; //snelheid
	//TIMSK2 |= (1 << 1); //enable interupt

	//init
	switchstatus[0] = 0xFF;
	switchstatus[1] = 0xFF;
	switchstatus[2] = 0xFF;

	switchcount = 0;

	MEM_read();

	RUN_home();
}
void RUN_home() { //move to HOME
	speed = 100;
	down;
	GPIOR0 |= (1 << 0);
	start;
	DSP_exe(10);
}

ISR(TIMER2_COMPA_vect) {
	if (GPIOR0 & (1 << 1)) {
		POS++;
	}
	else {
		POS--;
	}
	if (~GPIOR0 & (1 << 0)) {
		if (POS == POS_rq) {
			stop;
			Serial.println("stop");
		}
		if (POS == 0) {
			stop;
			//Serial.println("nul");
		}
	}
}
void FACTORY() {
	//clears eeprom
	Serial.println("factory");
	for (byte i = 0; i < 255; i++) {
		EEPROM.update(i, 0xFF);
	}
}
void MEM_read() {
	etage_status = EEPROM.read(100);
	for (byte i = 0; i < 8; i++) {
		EEPROM.get(0 + (5 * i), etage[i]);
		if (etage[i] == 0xFFFFFFFF)etage[i] = 0;
		//Serial.print(i); Serial.println("---"); Serial.println(etage[i]);
	}
}
void DSP_exe(byte txt) {
	cd;
	switch (txt) {
	case 10:
		regel1s; display.print("Zoek Home");
		break;
	case 12:
		//Serial.println(txt);		

		if (etage_status & (1 << etage_rq)) { //etage niet bepaald
			regel1s; display.print("Etage "); display.print(etage_rq); display.print(" niet bepaald");
			regelb; display.print("*");
		}
		else { //etage bepaald

			regelb; display.print(etage_rq);
		}
		break;
	case 15://keuze in te stellen etage
		regel1s; display.print("etage instellen ");
		regel2; display.print(etage_rq);
		break;
	case 16: //instellen etage
		regel1s; display.print("Instellen etage "); display.print(etage_rq);
		//Serial.println(POS);
		regel2; display.print(POS);
		break;
	case 20:
		break;
	}
	display.display();
}
void DSP_prg() {
	//Serial.print("PRG_fase: "); Serial.println(PRG_fase);

	switch (PRG_fase) {
	case 0: //in bedrijf
		DSP_exe(12);
		break;
	case 1: //bepaal etages
		switch (PRG_level) {
		case 0: //kiezen etage
			DSP_exe(15);
			break;
		case 1: //instellen gekozen etage
			DSP_exe(16);
			break;
		case 2: //max levels
			PRG_level = 0;
			DSP_exe(15);
			etage[etage_rq] = POS;
			etage_status &= ~(1 << etage_rq);
			EEPROM.put(0 + (5 * etage_rq), POS);
			EEPROM.update(100, etage_status);
		}
		break;
	}
}

void DSP_settxt(byte X, byte Y, byte size) {
	display.setTextSize(size);
	display.setTextColor(WHITE);
	if (X + Y > 0) display.setCursor(X, Y);
}

void SW_read() { //lezen van schakelaars
	//Dit werkt alleen met pullups naar 5V van 1K (interne pullup is te zwak)
	byte sr; byte changed;
	switchcount++;
	if (switchcount > 2)switchcount = 0;
	//Serial.println(switchcount);
	//PORTD |= (B11100000 << 0); //clear, set pins
	DDRD &= ~(B11100000); //set H-z
	switch (switchcount) {
	case 0:
		DDRD |= (1 << 7);
		PORTD &= ~(1 << 7); //set pin		
		break;
	case 1:
		DDRD |= (1 << 6);
		PORTD &= ~(1 << 6); //set pin
		break;
	case 2:
		DDRD |= (1 << 5);
		PORTD &= ~(1 << 5); //set pin
		break;
	}
	sr = PINC << 4;
	sr = sr >> 4;
	//sr |= (B11110000 << 0); //set bits 7~4
	changed = sr ^ switchstatus[switchcount];
	//Serial.println("*");
	for (byte i = 0; i < 4; i++) {
		if (changed & (1 << i)) {
			if (~sr & (1 << i)) {
				SW_on(i + 4 * switchcount);
			}
			else {
				SW_off(i + 4 * switchcount);
			}
		}
	}
	switchstatus[switchcount] = sr;
}
void SW_off(byte sw) {
	//Serial.print("uit: "); Serial.println(sw);
	switch (sw) {
	case 0:
		SW_0(false);
		break;
	case 1:
		SW_1(false);
		break;
	}
}
void SW_on(byte sw) {
	//Serial.println(sw);
	switch (sw) {
	case 0:
		SW_0(true);
		break;
	case 1:
		SW_1(true);
		break;
	case 2:
		PRG_level++;
		DSP_prg();
		break;
	case 3:
		SW_3();
		break;
	case 4:
		if (GPIOR0 & (1 << 0)) { //home gevonden
			stop;
			POS = 0;
			GPIOR0 &=~(1 << 0);
			//check request spot (default 0)
			etage_rq = 0;
			if (etage_status & (1 << etage_rq)) { //eerst alleen voor default
				DSP_exe(12);
			}
			else { // spot bekend ga naar positie
				POS_rq = etage[0];
				//Serial.println(POS_rq);
				if (POS_rq > 0) {
					up; start;
				}					
				DSP_prg();
			}
		}
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8://enc A
		if (switchstatus[2] & (1 << 1)) {
			GPIOR0 ^= (1 << 7);
			if (GPIOR0 & (1 << 7))SW_encoder(true);
		}
		else {
			GPIOR0 ^= (1 << 6);
			if (GPIOR0 & (1 << 6)) SW_encoder(false);
		}
		break;
	case 9: ///Enc B
		break;
	case 10://Enc switch
		PORTB ^= (1 << 1); //toggle enable poort
		break;
	case 11: //nc

		break;
	}
}void SW_0(boolean onoff) { //up
	if (onoff) {
		switch (PRG_fase) {
		case 0:
			stop;
			Serial.println(POS);
			break;
		case 1:
			switch (PRG_level) {
			case 0:
				break;
			case 1: //start motor
				up;
				start;
				break;
			}
			break;
		}
	}
	else {
		//1 situatie
		if (PRG_fase == 1 & PRG_level == 1) {
			stop;
			DSP_exe(16);
		}

	}
}
void SW_1(boolean onoff) { //down
	if (onoff) {
		switch (PRG_fase) {
		case 0:
			break;
		case 1:
			switch (PRG_level) {
			case 0:
				break;
			case 1: //start motor
				if (POS > 0) {
					down;
					start;
				}

				break;
			}
			break;
		}
	}
	else {
		//1 situatie
		if (PRG_fase == 1 & PRG_level == 1) {
			stop;
			DSP_exe(16);
		}
	}
}
void SW_3() {
	PRG_fase++;
	if (PRG_fase > 1)PRG_fase = 0;
	DSP_prg();


	//PORTB ^= (1 << 1); //enabled
	//TCCR2A ^= (1 << 6); //enable/disable timer
	/*
	TCCR2B ^= (1 << 3);
	if (~TCCR2B & (1 << 3)) {
		Serial.print("Aantal stappen: ");
		Serial.println(POS);
		POS = 0;
	}
*/

}
void SW_encoder(boolean dir) {
	//Serial.println(dir);
	switch (PRG_fase) {
	case 0:
		if (!dir) {
			etage_rq++;
			if (etage_rq > 7)etage_rq = 0;
		}
		else {
			etage_rq--;
			if (etage_rq > 7)etage_rq = 7;
		}
		POS_rq = etage[etage_rq];

		Serial.print("POS= "); Serial.println(POS);
		Serial.print("POS_rq= "); Serial.println(POS_rq);

		
		if (POS < POS_rq) {
			up;
		}
		else {
			down;
		}
		//Serial.print("GPIOR0 bit 0: "); Serial.println(GPIOR0, BIN);
		if (POS != POS_rq)start;

		//als autostart aanstaat, nog maken

		DSP_exe(12);
		break;
	case 1:
		if (~dir) {
			etage_rq++;
			if (etage_rq > 7)etage_rq = 0;
		}
		else {
			etage_rq--;
			if (etage_rq > 7)etage_rq = 7;
		}
		DSP_exe(15);
		break;
	}
}
void loop() {
	if (millis() - slowtime > 2) {
		slowtime = millis();
		SW_read();
	}

	//if (millis() - wait > 1000) {
	//	if (GPIOR0 & (1 << 2))start;
	//	GPIOR0 &= ~(1 << 2);
	//}
}
