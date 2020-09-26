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
#define regelb DSP_settxt(30,20,6)
#define regelst DSP_settxt(1,1,1)
#define speed OCR2A
#define up PORTB |=(1<<2);GPIOR0 |=(1<<1);
#define down PORTB &=~(1<<2);GPIOR0 &=~(1<<1);




//#define staart if(GPIOR0 TCCR2B |= (1 << 3);TIMSK2 |= (1 << 1); //enable interupt   //if(GPIOR0 & (1<<3){ 
//#define stop TCCR2B &=~(1 << 3);  TIMSK2 &=~(1 << 1); //disable interupt

Adafruit_SSD1306 display(128, 64, &Wire, -1);
byte slowcount;
unsigned int speedcount;
byte SPD_step;
volatile unsigned long POS;
volatile unsigned long POS_rq;
volatile unsigned long etage[8];
byte etage_status; //bit0 false positie bepaald, enz lezen eeprom 100
byte etage_rq;
byte MEM_reg;
byte switchstatus[3];
byte switchcount;
byte PRG_fase;
byte PRG_level;
byte ENC_count;
byte Vmax = 6;
volatile unsigned long SPD_dis;
volatile byte SPD_disstep;
void setup() {
	Serial.begin(9600);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	cd; //clear display
	DSP_settxt(10,30,1); display.print("www.wisselmotor.nl");
	//regel2; display.print("Treinlift");
	display.display();
	delay(2000);
	cd;
	regel2; display.print("TreinLift");
	display.display();
	delay(2000);
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
	TCCR2B |= (2 << 0); //prescaler standaard
	OCR2A = 255; //snelheid
	//TIMSK2 |= (1 << 1); //enable interupt
	//init
	switchstatus[0] = 0xFF;
	switchstatus[1] = 0xFF;
	switchstatus[2] = 0xFF;
	switchcount = 0;
	MEM_read();
	MOTOR();
	RUN_home();
}
void start() {
	if (GPIOR0 & (1 << 3)) {
		SPD_disstep = 13;
		SPD_step = 6;
		if (MEM_reg & (1 << 0)) SPD_step = 0;
		GPIOR0 |= (1 << 4);
		if (PRG_fase > 0)SPD_step = 6;
		SPD(SPD_step);
		TCCR2B |= (1 << 3);
		TIMSK2 |= (1 << 1); //enable interupt 
	}
}
void stop() {
	TCCR2B &= ~(1 << 3);
	TIMSK2 &= ~(1 << 1);
	GPIOR0 &= ~(1 << 4);
}
void MOTOR() {
	GPIOR0 ^= (1 << 3);
	if (~GPIOR0 & (1 << 3)) {
		PORTB |= (1 << 1);
		stop();
		DSP_exe(20);
	}
	else {
		PORTB &= ~(1 << 1);
		if (~GPIOR0 & (1 << 0)) {
			DSP_exe(21);
		}
		else {
			RUN_home();
		}
	}
}
void RUN_home() { //move to HOME	
	down;
	GPIOR0 |= (1 << 0);
	start();
	DSP_exe(10);	
}
ISR(TIMER2_COMPA_vect) {
	int sd;	
	if (POS_rq > POS) {
		sd = POS_rq - POS;
	}
	else {
		sd = POS - POS_rq;
	}

	if (GPIOR0 & (1 << 1)) {
		POS++;
	}
	else {
		POS--;
	}
	if (~GPIOR0 & (1 << 0)) {
		if (POS == POS_rq) {
			stop();
			Serial.println("stop");
		}
		if (POS == 0) {
			stop();
			//Serial.println("nul");
		}
	}
	//vertraging
		if (MEM_reg & (1 << 0) & PRG_fase ==0) {
			if (~GPIOR0 & (1 << 0)) { //niet tijdens home
				if (sd < SPD_dis) {
					GPIOR0 &= ~(1 << 4); //stop versnellen		
					if (SPD_disstep > SPD_step) {
						SPD_step--;
						SPD_disstep--;
						SPD(SPD_step);
					}
				}
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
	Vmax = EEPROM.read(101);
	MEM_reg = EEPROM.read(102);
	if (Vmax > 12) Vmax = 6;
	for (byte i = 0; i < 8; i++) {
		EEPROM.get(0 + (5 * i), etage[i]);
		if (etage[i] == 0xFFFFFFFF)etage[i] = 0;
	}
}
void SPD(byte step) {
	//Serial.print("step: "); Serial.println(step);// Serial.print("  Pos: "); Serial.println(POS);
	switch (step) {
	case 0:
		//TCCR2B |= (7 << 0);
		SPD_dis = 100;
		speed = 255;
		break;
	case 1:
		//TCCR2B |= (2 << 0);
		SPD_dis = 250;
		speed = 220;
		break;
	case 2:
		//TCCR2B |= (2 << 0);
		SPD_dis = 500;
		speed = 180;
		break;
	case 3:
		SPD_dis = 700;
		speed = 150;
		break;
	case 4:
		//TCCR2B |= (2 << 0);
		SPD_dis = 900;
		speed = 120;
		break;
	case 5:
		SPD_dis = 1100;
		speed = 90;
		break;
	case 6:
		SPD_dis = 1250;
		speed = 70;
		break;
	case 7:
		SPD_dis = 1500;
		speed = 60;
		break;
	case 8:
		SPD_dis = 1800;
		speed = 50;
		break;
	case 9:
		SPD_dis = 2100;
		speed = 40;
		break;
	case 10:
		SPD_dis = 2400;
		speed = 30;
		break;
	case 11:
		SPD_dis = 2700;
		speed = 20;
		break;
	case 12:
		SPD_dis = 3000;
		speed = 10;
		break;
	}
}
void DSP_exe(byte txt) {
	cd;
	switch (txt) {
	case 10:
		regelst; display.print("going home.....");
		break;
	case 12:
		if (etage_status & (1 << etage_rq)) { //etage niet bepaald
			regel1s; display.print("Etage "); display.print(etage_rq); display.print(" niet bepaald");
			regelb; display.print("*");
		}
		else { //etage bepaald
			regelb; display.print(etage_rq);
			if (~GPIOR0 & (1 << 3)) { //motor uit
				regelst; display.print("Stop");
			}
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
	case 20://motor off
		regelb; display.print("X");
		break;
	case 21:
		regelb; display.print("-");
		break;
	case 30://handmatig instellen
		regel1s; display.print("Handmatig, positie");
		regel2; display.print(POS);
		break;
	case 40:
		regel1s; display.print("max snelheid");
		regel2; display.print(Vmax);
		break;
	case 50:
		regel1s; display.print("Diverse ");
		switch (PRG_level) {
		case 0:
			display.print("versnellen"); regel2;
			if (MEM_reg & (1 << 0)) {
				display.print("Ja");
			}
			else {
				display.println("nee");
			}
			break;
		}
		break;
	}
	display.display();
}
void DSP_prg() {
	switch (PRG_fase) {
	case 0: //in bedrijf
		DSP_exe(21); //12
		break;
	case 1://handmatig
		DSP_exe(30);
		break;
	case 2: //bepaal etages
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
			break;
		}
		break;
	case 3: //Vmax
		DSP_exe(40);
		break;
	case 4: //diverse instellingen
		DSP_exe(50);
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
	byte sr; byte changed; byte v;
	switchcount++;
	if (GPIOR0 & (1 << 0)) {
		v = 4;
	}
	else {
		v = Vmax;
	}
	if (MEM_reg & (1 << 0) & PRG_fase == 0) {
		if (GPIOR0 & (1 << 4)) {//versnellen
			speedcount++;
			if (speedcount > 500) {
				speedcount = 0;
				if (SPD_step < v) {
					SPD_step++;
					SPD(SPD_step);
				}
				else {
					GPIOR0 &= ~(1 << 4);
				}
			}
		}
	}
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
	case 8: //encoder A
		switch (ENC_count) {
		case 2:
			ENC_count = 3;
			break;
		case 13:
			ENC_count = 0;
			SW_encoder(false);
			break;
		default:
			ENC_count = 0;
			break;
		}
		break;
	case 9: //encoder B
		switch (ENC_count) {
		case 3:
			ENC_count = 0;
			SW_encoder(true);
			break;
		case 12:
			ENC_count = 13;
			break;
		default:
			ENC_count = 0;
			break;
		}
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
		SW_2();
		break;
	case 3:
		SW_3();
		break;
	case 4:
		if (GPIOR0 & (1 << 0)) { //home gevonden
			stop();
			POS = 0;
			GPIOR0 &= ~(1 << 0);
			//check request spot (default 0)
			etage_rq = 0;
			if (etage_status & (1 << etage_rq)) { //eerst alleen voor default
				DSP_exe(12);
			}
			else { // spot bekend ga naar positie
				POS_rq = etage[0];
				//Serial.println(POS_rq);
				if (POS_rq > 0) {
					up; start();
				}
				DSP_exe(12);//DSP_prg();
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
		switch (ENC_count) {
		case 0:
			ENC_count = 1;
			break;
		case 11:
			ENC_count = 12;
			break;
		default:
			ENC_count = 0;
			break;
		}

		/*
		if (switchstatus[2] & (1 << 1)) {
			GPIOR0 ^= (1 << 7);
			if (GPIOR0 & (1 << 7))SW_encoder(true);
		}
		else {
			GPIOR0 ^= (1 << 6);
			if (GPIOR0 & (1 << 6)) SW_encoder(false);
		}

*/
		break;
	case 9: ///Enc B
		switch (ENC_count) {
		case 0:
			ENC_count = 11;
			break;
		case 1:
			ENC_count = 2;
			break;
		default:
			ENC_count = 0;
			break;
		}
		break;
	case 10://Enc switch
		MOTOR(); //toggle motor on/off
		//PORTB ^= (1 << 1); //toggle enable poort
		break;
	case 11: //nc

		break;
	}
}void SW_0(boolean onoff) { //up
	if (onoff) {
		switch (PRG_fase) {
		case 0:
			SW_encoder(true);
			break;
		case 1: //handmatig
			up;
			start();
			break;
		case 2: //instellen etages
			switch (PRG_level) {
			case 0:
				SW_encoder(true);
				break;
			case 1: //start motor
				up;
				start();
				break;
			}
			break;
		case 3:
			SW_encoder(true);
			break;
		case 4:
			switch (PRG_level) {
			case 0:
				MEM_reg ^= (1 << 0);
				DSP_exe(50);
				break;
			case 1:
				break;
			}

			break;
		}
	}
	else {
		//1 situatie
		switch (PRG_fase) {
		case 1:
			stop();
			DSP_exe(30);
			break;
		case 2:
			if (PRG_level == 1) {
				stop();
				DSP_exe(16);
			}
			break;
		}
	}
}
void SW_1(boolean onoff) { //down
	if (onoff) {
		switch (PRG_fase) {
		case 0:
			SW_encoder(false);
			break;
		case 1: //handmatig
			if (POS > 0) {
				down;
				start();
			}
			break;
		case 2: //instellen etage
			switch (PRG_level) {
			case 0:
				SW_encoder(false);
				break;
			case 1: //start motor
				if (POS > 0) {
					down;
					start();
				}

				break;
			}
			break;
		case 3: //instellen Vmax
			SW_encoder(false);
			break;
		case 4: //diverse
			//verhogen level 
			break;
		}
	}
	else {
		switch (PRG_fase) {
		case 1:
			stop();
			DSP_exe(30);
			break;
		case 2:
			if (PRG_level == 1) {
				stop();
				DSP_exe(16);
			}
			break;
		}

	}
}
void SW_2() {
	switch (PRG_fase) {
	case 0: //n bedrijf
		MOTOR();
		break;
	case 2: //installen etages
		PRG_level++;
		DSP_prg();
		break;
	case 3: //instellen Vmax
		EEPROM.update(101, Vmax);
		PRG_fase = 0;
		DSP_prg();
		break;
	case 4://diverse
		EEPROM.update(102, MEM_reg);
		PRG_fase = 0;
		DSP_prg();
		break;
	}
}
void SW_3() {
	PRG_fase++;
	if (PRG_fase > 4)PRG_fase = 0;
	DSP_prg();
	PRG_level = 0;
}
void SW_encoder(boolean dir) {
	//Serial.println(dir);
	switch (PRG_fase) {
	case 0:
		ENC_select(dir);
		POS_rq = etage[etage_rq];
		if (POS < POS_rq) {
			up;
		}
		else {
			down;
		}
		if (POS != POS_rq)start();
		DSP_exe(12);
		break;
	case 1: //handmatig
		ENC_fine(dir);
		DSP_exe(30);
		break;
	case 2: //etage instellen
		switch (PRG_level) {
		case 0:
			ENC_select(dir);
		DSP_exe(15);
			break;
		case 1:
			ENC_fine(dir);
			DSP_exe(16);
			break;
		}
		break;
	case 3: //vmax instellen
		if (dir) {
			if (Vmax > 1)Vmax--;
		}
		else {
			if (Vmax < 12) Vmax++;
		}
		DSP_exe(40);
		break;
	}
}
void ENC_fine(boolean dir) {
	if (dir) {
		down;
		PORTB ^= (1 << 3);
		if (PINB & (1 << 3))POS--;
	}
	else {
		up;
		PORTB ^= (1 << 3);
		if (PINB & (1 << 3))POS++;
	}
}
void ENC_select(boolean dir) {
	if (!dir) {
		etage_rq++;
		if (etage_rq > 7)etage_rq = 0;
	}
	else {
		etage_rq--;
		if (etage_rq > 7)etage_rq = 7;
	}
}
void loop() {
	slowcount++;
	if (slowcount == 0xFF) {
		SW_read();
	}
}
