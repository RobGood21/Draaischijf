/*
 Name:		TreinLift.ino
 Created:	9/22/2020 10:13:11 AM
 Author:	rob Antonisse

 Design for train vertical fiddle yard. Treinlift
 Steppermotor 23HS5628-08 stepper driver TB6600


 V1.02 Snelheid opgevoerd, standaard home speed en overall speed x2 verhouding in snelheidsstappen verder uiteen gehaald
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
#define regelb DSP_settxt(50,10,6)
#define regelst DSP_settxt(1,1,1)
#define up PORTB |=(1<<2);GPIOR0 |=(1<<1);
#define down PORTB &=~(1<<2);GPIOR0 &=~(1<<1);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

//**Declaraties for DeKoder
volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts
boolean DEK_Monitor = false; //shows DCC commands as bytes
byte DEK_Reg; //register voor de decoder 
byte DEK_Status = 0;
byte DEK_byteRX[6]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte DEK_countPA = 0; //counter for preample
byte DEK_BufReg[6]; //registerbyte for 12 command buffers, can be lowered.
byte DEK_Buf0[6];
byte DEK_Buf1[6];
byte DEK_Buf2[6];
byte DEK_Buf3[6];
byte DEK_Buf4[6];
byte DEK_Buf5[6];
//**End declaration for deKoder
byte DCC_adres;

byte COM_reg;
byte slowcount;
volatile unsigned long POS;
volatile unsigned long POS_rq;
volatile unsigned long etage[8];
byte etage_status; //bit0 false positie bepaald, enz lezen eeprom 100
byte etage_rq;
byte switchstatus[3];
byte switchcount;
byte PRG_fase;
byte PRG_level;
byte ENC_count;
byte Vhome;
byte Vmin;
byte Vmax;

byte count;
byte RPM_time;
volatile unsigned long RPM_pos;
unsigned long runwait;

void setup() {
	Serial.begin(9600);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

	//**begin Setup for DeKoder 
	//Serial.begin(9600);
	//interrupt on PIN2
	DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//External Interrupt Mask Register bit0 INT0 > 1
	//**End Setup for DeKoder


	cd; //clear display
	DSP_settxt(10, 30, 1); display.print("www.wisselmotor.nl");
	//regel2; display.print("Treinlift");
	display.display();
	delay(500);
	cd;
	regel2; display.print("TreinLift");
	display.display();
	delay(500);
	//ports
	PORTC |= (15 << 0); //pullup  A0~A3
	DDRB |= (15 << 0); //Pin8~Pin11 as outputs
	PINB |= (1 << 0); //set pin8
	DDRD |= (B11100000 << 0); //pin 7,6,5 output
	//factory reset
	DDRD |= (1 << 7);
	Serial.println(PINC);
	if (PINC == 54) {
		FACTORY();
		delay(500);
	}
	//timer 2 settings on pin11 compare match
	TCCR2A |= (1 << 6); //toggle pin6
	TCCR2A |= (1 << 1);


	//geen instelling prescaler(TCCR2B bits 0,1,2) = 2x sneller
	TCCR2B |= (1 << 0); //afstelling V1.03
	//TCCR2B |= (1 << 1); //2x langzamer


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
ISR(INT0_vect) { //ISR for DeKoder receive on Pin 2
	//DEK_Reg fase van bit ontvangst
	//bit 0= bitpart ok (1) of failed(0)
	//bit1= truepart 
	//bit2=falsepart
	//bit3= received bit true =true false =false
	//bit4=restart, begin, failed as true

	cli();
	DEK_duur = (micros() - DEK_Tperiode);
	DEK_Tperiode = micros();
	if (DEK_duur > 50) {
		if (DEK_duur < 62) {
			DEK_Reg |= (1 << 0); //bitSet(DekReg, 0);

			if (bitRead(DEK_Reg, 1) == false) {
				DEK_Reg &= ~(1 << 2); //bitClear(DekReg, 2);
				DEK_Reg |= (1 << 1);
			}
			else { //received full true bit

				DEK_Reg |= (1 << 3);
				DEK_BitRX();
				DEK_Reg &= ~(1 << 2);//bitClear(DekReg, 2);
				DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
			}
		}
		else {
			if (DEK_duur > 106) {

				if (DEK_duur < 124) { //preferred 118 6us extra space in false bit
					DEK_Reg |= (1 << 0); //bitSet(DekReg, 0);
					if (bitRead(DEK_Reg, 2) == false) {
						DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
						DEK_Reg |= (1 << 2);  //bitSet(DekReg, 2);
					}
					else { //received full false bit
						DEK_Reg &= ~(1 << 3); //bitClear(DekReg, 3);
						DEK_BitRX();
						DEK_Reg &= ~(1 << 2);//bitClear(DekReg, 2);
						DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
					}
				}
			}
		}
	}
	sei();
}
//********Begin Void's for DeKoder
void DEK_begin() {//runs when bit is corrupted, or command not correct
	//lesscount++;
	DEK_countPA = 0;
	DEK_Reg = 0;
	DEK_Status = 0;
	for (int i = 0; i < 6; i++) {
		DEK_byteRX[i] = 0; //reset receive array
	}
}
void DEK_BufCom(boolean CV) { //create command in Buffer
	byte i = 0;
	while (i < 12) {

		if (bitRead(DEK_BufReg[i], 7) == false) {
			DEK_BufReg[i] = 0; //clear found buffer


			DEK_Buf0[i] = DEK_byteRX[0];
			DEK_Buf1[i] = DEK_byteRX[1];
			DEK_Buf2[i] = DEK_byteRX[2];

			if (CV == true) {
				DEK_BufReg[i] |= (1 << 0); //set for CV
				DEK_Buf3[i] = DEK_byteRX[3];
				DEK_Buf4[i] = DEK_byteRX[4];
				DEK_Buf5[i] = DEK_byteRX[5];
			}
			else {

				DEK_Buf3[i] = 0;
				DEK_Buf4[i] = 0;
				DEK_Buf5[i] = 0;
			}
			DEK_BufReg[i] |= (1 << 7); //claim buffer
			i = 15;
		}
		i++;
	} //close for loop
} //close void
void DEK_BitRX() { //new version
	static byte countbit = 0; //counter received bits
	static byte countbyte = 0;
	static byte n = 0;
	DEK_Reg |= (1 << 4);//resets and starts process if not reset in this void
	switch (DEK_Status) {
		//*****************************
	case 0: //Waiting for preample 
		if (bitRead(DEK_Reg, 3) == true) {
			DEK_countPA++;
			if (DEK_countPA > 12) {
				DEK_Status = 1;
				countbit = 0;
				countbyte = 0;
			}
			bitClear(DEK_Reg, 4);
		}
		break;
		//*************************
	case 1: //Waiting for false startbit
		if (bitRead(DEK_Reg, 3) == false) { //startbit receive
			DEK_countPA = 0;
			DEK_Status = 2;
		}
		//if Dekreg bit 3= true no action needed.
		bitClear(DEK_Reg, 4); //correct, so resume process
		break;
		//*************************
	case 2: //receiving data
		if (bitRead(DEK_Reg, 3) == true) DEK_byteRX[countbyte] |= (1 << (7 - countbit));
		countbit++;
		if (countbit == 8) {
			countbit = 0;
			DEK_Status = 3;
			countbyte++;
		}
		bitClear(DEK_Reg, 4); //correct, so resume process
		break;
		//*************************
	case 3: //waiting for separating or end bit
		if (bitRead(DEK_Reg, 3) == false) { //false bit
			DEK_Status = 2; //next byte
			if ((bitRead(DEK_byteRX[0], 6) == false) & (bitRead(DEK_byteRX[0], 7) == true))bitClear(DEK_Reg, 4); //correct, so resume process	
		}
		else { //true bit, end bit, only 3 byte and 6 byte commands handled by this dekoder
			switch (countbyte) {
			case 3: //Basic Accessory Decoder Packet received
				//check error byte
				if (DEK_byteRX[2] = DEK_byteRX[0] ^ DEK_byteRX[1])DEK_BufCom(false);
				break; //6
			case 6: ///Accessory decoder configuration variable Access Instruction received (CV)
				//in case of CV, handle only write command
				if (bitRead(DEK_byteRX[2], 3) == true && (bitRead(DEK_byteRX[2], 2) == true)) {
					//check errorbyte and make command
					if (DEK_byteRX[5] = DEK_byteRX[0] ^ DEK_byteRX[1] ^ DEK_byteRX[2] ^ DEK_byteRX[3] ^ DEK_byteRX[4])DEK_BufCom(true);
				}
				break;
			} //close switch bytecount
		}//close bittype
		break;
		//***************************************
	} //switch dekstatus
	if (bitRead(DEK_Reg, 4) == true)DEK_begin();
}
void DEK_DCCh() { //handles incoming DCC commands, called from loop()
	static byte n = 0; //one buffer each passing
	byte temp;
	int decoder;
	int channel = 1;
	int adres;
	boolean port = false;
	boolean onoff = false;
	int cv;
	int value;

	//translate command
	if (bitRead(DEK_BufReg[n], 7) == true) {
		decoder = DEK_Buf0[n] - 128;
		if (bitRead(DEK_Buf1[n], 6) == false)decoder = decoder + 256;
		if (bitRead(DEK_Buf1[n], 5) == false)decoder = decoder + 128;
		if (bitRead(DEK_Buf1[n], 4) == false)decoder = decoder + 64;
		//channel
		if (bitRead(DEK_Buf1[n], 1) == true) channel = channel + 1;
		if (bitRead(DEK_Buf1[n], 2) == true) channel = channel + 2;
		//port
		if (bitRead(DEK_Buf1[n], 0) == true)port = true;
		//onoff
		if (bitRead(DEK_Buf1[n], 3) == true)onoff = true;
		//CV
		if (bitRead(DEK_BufReg[n], 0) == true) {
			cv = DEK_Buf3[n];
			if (bitRead(DEK_Buf2[n], 0) == true)cv = cv + 256;
			if (bitRead(DEK_Buf2[n], 1) == true)cv = cv + 512;
			cv++;
			value = DEK_Buf4[n];
		}
		else {
			cv = 0;
			value = 0;
		}
		COM_exe(bitRead(DEK_BufReg[n], 0), decoder, channel, port, onoff, cv, value);
		//Show Monitor (bytes)
		if (DEK_Monitor == true) {
			Serial.print("buffer= ");
			Serial.print(n);
			Serial.print("  value:  ");
			Serial.print(bitRead(DEK_BufReg[n], 7));
			Serial.print(bitRead(DEK_BufReg[n], 6));
			Serial.print(bitRead(DEK_BufReg[n], 5));
			Serial.print(bitRead(DEK_BufReg[n], 4));
			Serial.print(bitRead(DEK_BufReg[n], 3));
			Serial.print(bitRead(DEK_BufReg[n], 2));
			Serial.print(bitRead(DEK_BufReg[n], 1));
			Serial.print(bitRead(DEK_BufReg[n], 0));
			Serial.println("");

			temp = DEK_Buf0[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf1[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf2[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf3[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf4[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");


			temp = DEK_Buf5[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");
			Serial.println("------");

		}
		//clear buffer
		DEK_BufReg[n] = 0;
		DEK_Buf0[n] = 0;
		DEK_Buf1[n] = 0;
		DEK_Buf2[n] = 0;
		DEK_Buf3[n] = 0;
		DEK_Buf4[n] = 0;
		DEK_Buf5[n] = 0;
	}
	n++;
	if (n > 6)n = 0;
}
void COM_exe(boolean type, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	int adres;
	adres = ((decoder - 1) * 4) + channel;
	//Applications 
	//APP_Monitor(type, adres, decoder, channel, port, onoff, cv, value);
	APP_DCC(type, adres, decoder, channel, port, onoff, cv, value);
	//Add a void like APP_monitor for application
}
//**End void's for DeKoder
void APP_DCC(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	byte n;
	n = decoder - DCC_adres;
	switch (n) {
	case 0:
		if (port == true) {
			etage_rq = channel - 1;
			ET_rq();
			DSP_exe(12);
			//Serial.println(channel-1);
		}
		break;
	case 1:
		if (port == true) {
			etage_rq = channel + 3;
			ET_rq();
			DSP_exe(12);
			//Serial.println(channel + 3);
		}
		break;
	}
}
void start() {
	RPM_time = 0;
	if (GPIOR0 & (1 << 3)) { //motor aan
		OCR2A = Vhome;
		if (~GPIOR0 & (1 << 0)) { //naar home
			if (PRG_fase == 0) OCR2A = Vmin; //versnellen/vertragen instelling er nog ij zetten
		}
		GPIOR0 |= (1 << 4); //versnellen	
		RPM_pos = 50000;
		//start motor
		TCCR2B |= (1 << 3);
		TIMSK2 |= (1 << 1); //enable interupt 
		GPIOR0 |= (1 << 5);
		PORTB |= (1 << 0);//bezet led
	}
}
void stop() {
	TCCR2B &= ~(1 << 3);
	TIMSK2 &= ~(1 << 1);
	GPIOR0 &= ~(1 << 4);
	GPIOR0 &= ~(1 << 5);
	PORTB &= ~(1 << 0); //bezetled 
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
		RUN_home();
	}
}
void RUN_home() { //move to HOME	
	down;
	GPIOR0 |= (1 << 0); //home zoeken
	start();
	DSP_exe(10);
}
ISR(TIMER2_COMPA_vect) {
	unsigned int dis;
	if (GPIOR0 & (1 << 1)) { //true is omlaag
		POS++;
	}
	else { //false is omlaag
		POS--;
	}
	if (~GPIOR0 & (1 << 0)) { //niet home 
		if (POS == POS_rq) {
			stop();
			//Serial.println("stop");
		}
		if (POS == 0) {
			stop();
			//Serial.println("nul");
		}

		if (PRG_fase == 0) { //alleen bij in bedrijf, instelling nog erbij		

			if (GPIOR0 & (1 << 4))FAST(); //versnellen aangezet in start

			if (GPIOR0 & (1 << 1)) { //omhoog vertraging berekenen
				if (POS_rq - RPM_pos == POS) {
					GPIOR0 &= ~(1 << 4);
					RPM_pos = RPM_pos - (RPM_pos / (Vmin - OCR2A));
					RPM_pos = RPM_pos - ((Vmin - OCR2A) * 5);
					OCR2A++;
				}
			}
			else { //omlaag, vertraging berekenen
				if (POS_rq + RPM_pos == POS) {
					GPIOR0 &= ~(1 << 4);
					RPM_pos = RPM_pos - (RPM_pos / (Vmin - OCR2A));
					RPM_pos = RPM_pos - ((Vmin - OCR2A) * 5);
					OCR2A++;
				}

			}
		}

	}
}
void FAST() {
	unsigned long dis;
	if (GPIOR0 & (1 << 1)) {
		dis = POS_rq - POS;
	}
	else {
		dis = POS - POS_rq;
	}
	if (dis > RPM_pos) {
		if (OCR2A > Vmax) {
			//RPM_count ++;
			//if (RPM_count >10) { //Vmin - OCR2A){ //RPM_time) { // 1/4 rotatie
				//RPM_count = 0;
			if (GPIOR1 & (1 << 0)) {
				OCR2A--;
				GPIOR1 &= ~(1 << 0);
			}
		}
	}
	else {
		OCR2A = Vhome;
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

	Vhome = EEPROM.read(110);
	if (Vhome == 0xFF)Vhome = 20;
	Vmin = EEPROM.read(111);
	if (Vmin == 0xFF)Vmin = 100;
	Vmax = EEPROM.read(112);
	if (Vmax == 0xFF)Vmax = 5;

	//MEM_reg = EEPROM.read(102);
	
	DCC_adres = EEPROM.read(103);
	if (DCC_adres == 0xFF)DCC_adres = 1;

	for (byte i = 0; i < 8; i++) {
		EEPROM.get(0 + (5 * i), etage[i]);
		if (etage[i] == 0xFFFFFFFF) {
			if (i == 0) {
				etage[i] = 20000;
			}
			else {
				etage[i] = 20000 + (i * 420000);
			}
		}
	}

	etage_status = EEPROM.read(100);
	if (etage_status == 0xFF)etage_status = 0;
}
void DSP_exe(byte txt) {
	EIMSK &= ~(1 << INT0); //interrupt DCC ontvangst ff uit
	cd;
	byte et;
	et = etage_rq + 1;
	switch (txt) {
	case 10:
		regelst; display.print("going home.....");
		break;
	case 12:
		if (etage_status & (1 << etage_rq)) { //etage niet bepaald
			regel1s; display.print("Etage "); display.print(et); display.print(" niet bepaald");
			regelb; display.print("*");
		}
		else { //etage bepaald
			regelb; display.print(et);
			if (~GPIOR0 & (1 << 3)) { //motor uit
				regelst; display.print("Stop");
			}
		}
		break;
	case 15://keuze in te stellen etage
		regel1s; display.print("etage instellen ");
		regel2; display.print(et);
		break;
	case 16: //instellen etage
		regel1s; display.print("Instellen etage "); display.print(et);
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
		regel1s; display.print("Snelheid"); regel2;
		switch (PRG_level) {
		case 0:
			display.print("Vhome: "); display.print(Vhome);
			break;
		case 1:
			display.print("Vmin: "); display.print(Vmin);
			break;
		case 2:
			display.print("Vmax: "); display.print(Vmax);
			break;
		}

		break;
	case 50:
		regel1s; display.print("Diverse ");
		switch (PRG_level) {
		case 0:
			display.print("");display.print("Geen functie");	
			break;
		}
		break;
	case 60:
		regel1s; display.println("DCC adres");
		regel2; display.print(DCC_adres * 4 - 3); regel2s; display.print("("); display.print(DCC_adres); display.print("-1~");
		display.print(DCC_adres + 1); display.print("-4)");
		break;
	}


	display.display();
	EIMSK |= (1 << INT0);
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
	case 5: //DCC adres instellen
		DSP_exe(60);
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
	if (switchcount > 2)switchcount = 0;
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
}
void SW_0(boolean onoff) { //up
	if (onoff) {
		switch (PRG_fase) {
		case 0: //in bedrijf
			SW_encoder(true);
			break;
		case 1: //handmatig instellen
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
		case 3: //snelheid instellen
			PRG_level++;
			if (PRG_level > 2)PRG_level = 0;
			DSP_exe(40);
			break;
		case 4:
			switch (PRG_level) {
			case 0:				
				DSP_exe(50);
				break;
			case 1:
				break;
			}

			break;
		case 5:
			SW_encoder(true);
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
		case 3: //instellen snelheid
			SW_encoder(false);
			break;
		case 4: //diverse
			//verhogen level 
			break;
		case 5:
			SW_encoder(false);
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
	case 2: //instellen etages
		PRG_level++;
		DSP_prg();
		break;
	case 3: //instellen snelheden
		EEPROM.update(110, Vhome);
		EEPROM.update(111, Vmin);
		EEPROM.update(112, Vmax);
		OCR2A = Vhome;
		PRG_fase = 0;
		DSP_prg();
		break;
	case 4://diverse		
		PRG_fase = 0;
		DSP_prg();
		break;
	case 5:
		EEPROM.update(103, DCC_adres);
		PRG_fase = 0;
		DSP_prg();
		break;
	}
}
void SW_3() {
	PRG_fase++;
	if (PRG_fase > 5)PRG_fase = 0;
	DSP_prg();
	PRG_level = 0;
}
void ET_rq() {
	//Serial.println("*");
	if (~COM_reg & (1 << 0)) {
		COM_reg |= ((1 << 0));
		runwait = millis();
	}
	else {
		if (millis() - runwait > 3000) {
			RUN_rq();
			COM_reg &= ~(1 << 0);
		}
	}
}
void RUN_rq() {
	if (GPIOR0 & (1 << 5)) {
		if (~GPIOR0 & (1 << 6)) {
			GPIOR0 |= (1 << 6);
		}
	}
	else {
		GPIOR0 &= ~(1 << 6);
		POS_rq = etage[etage_rq];
		if (POS < POS_rq) {
			up;
		}
		else {
			down;
		}
		if (POS != POS_rq)start();
	}
}
void SW_encoder(boolean dir) {
	switch (PRG_fase) {
	case 0:
		ENC_select(dir);
		ET_rq();
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
	case 3: //snelheid instellen
		if (dir) {
			switch (PRG_level) {
			case 0:
				Vhome--;
				break;
			case 1:
				Vmin--;
				break;
			case 2:
				Vmax--;
				break;
			}
		}
		else {
			switch (PRG_level) {
			case 0:
				Vhome++;
				break;
			case 1:
				Vmin++;
				break;
			case 2:
				Vmax++;
				break;
			}
		}
		DSP_exe(40);
		break;
	case 5: //DCC (decoder)adres
		if (dir) {
			DCC_adres--; if (DCC_adres < 1)DCC_adres = 250;
		}
		else {
			DCC_adres++; if (DCC_adres > 250)DCC_adres = 1;
		}

		DSP_exe(60);
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
	//tbv dekoder
	DEK_DCCh();
	slowcount++;
	if (slowcount == 0XFF) {
		count++;
		if (count == 0) {
			if (GPIOR0 & (1 << 6))RUN_rq();
			if (COM_reg & (1 << 0))ET_rq();
		}
		SW_read();
		RPM_time++;
		if (RPM_time > 10 + (Vmin - OCR2A)) {
			GPIOR1 |= (1 << 0);
			RPM_time = 0;
		}
	}
}
