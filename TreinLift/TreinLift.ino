/*
 Name:		TreinLift.ino
 Created:	9/22/2020 10:13:11 AM
 Author:	rob Antonisse

 Design for train vertical fiddle yard. Treinlift
 Steppermotor 23HS5628-08 stepper driver TB6600


 V1.02 Snelheid opgevoerd, standaard home speed en overall speed x2 verhouding in snelheidsstappen verder uiteen gehaald
 V2.01
 gehele snelheid methodiek veranderd. Mem_reg diverse, versnellen/vertragen verwijderd.
 Factory reset aangepast vult automatisch de 8 etages. (onegeveer 10cm bij 1.5mm per rotatie.
 Motor snelheid afgeregeld op full microstep 6400 steps voor 1 rotatie
 Schakelaar en encoder leessnelheid gehalveerd, meer tegengaan denderen contacten
 Toegevoegd in menu instellingen voor Vhome, Vmin
 Snelheid wordt in display geinverteerd weergeven
 Library splah.h en wire.h uitgezet
 V2.02
 Versie hardware op baseren project TurN V1.01
 Toegevoegd uitgang nog te definieren, tijdelijk op lock, als brug nieuwe positie vraag krijgt
 gaat uitgang laag als brug in rust is gaat uitgang aan.
 Vstep toegevoegd voor in runtime instellen van prescaler 9snelheid stappen)
 Etages hernoemd naar STOPS en aantal instelbaar gemaakt van 1~16

 Mem_read en factory reset aangepast voor versie bedoeld voor treinlift, aangedreven door draadeind.

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
//boolean DEK_Monitor = false; //shows DCC commands as bytes
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
byte DCC_mode; //EEPROM 
byte COM_reg;
byte slowcount;
unsigned long POS;
unsigned long POS_rq;
unsigned long POS_acc; //aantal stappen tussen afrem posities
unsigned long POS_calc; //afrem positie
volatile unsigned long stops[16];
byte aantalStops;
byte stops_status; //bit0 false positie bepaald, enz lezen eeprom 100
byte stops_rq;
byte switchstatus[3];
byte switchcount;
byte PRG_fase;
byte PRG_level;
byte ENC_count;
byte Vhome;
byte Vmin;
byte Vmax;
byte Vstep; //instelling prescaler 64;128;259;1024
byte count;
unsigned long runwait;
byte Vaccel = 3; //ingestelde acceleratietijd in seconden
float accStep; //berekende tijd per snelheids step in microsec
float puls;
unsigned long accTijd; //verstreken tijd

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
	DSP_settxt(10, 30, 1); display.print(F("www.wisselmotor.nl"));
	//regel2; display.print("Treinlift");
	display.display();
	delay(500);
	cd;
	regel2; display.print(F("TreinLift"));
	display.display();
	delay(500);
	//ports
	PORTC |= (15 << 0); //pullup  A0~A3
	DDRB |= (15 << 0); //Pin8~Pin11 as outputs
	PINB |= (1 << 0); //set pin8
	DDRD |= (B11110000 << 0); //pin 7,6,5,4 output
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
	TCCR2B |= (1 << 2); //prescaler always 4  on(default true???)
	OCR2A = 255; //snelheid
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

		/*
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
		*/



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
	//Als motor draait geen DCC ontvangst. Start() disables DCC Stop() enables DCC (EIMKS masks register)
	//Filter, alleen er toe doende commandoos doorlaten, als motor draait geen DCC ontvangst
	int ad;
	ad = (decoder - DCC_adres) * 4 + channel; //adressen omgezet in 0~16
	switch (DCC_mode) {
	case 0: //Single alleen true port schakeld stops
		if (ad < aantalStops) {
			if (port == true) {
				stops_rq = ad - 1;
				ET_rq();
				DSP_exe(12);
			}
		}
		break;
	case 1: //DUO-RA true schakeld stop false schakeld stop+1
		if (port == true) {
			ad = ((ad * 2) - 1) - 1;
		}
		else { //port =false
			ad = ((ad * 2)) - 1;
		}
		if (ad < aantalStops) {
			stops_rq = ad;
			ET_rq();
			DSP_exe(12);
		}
		break;
	case 2: //DUO AR true schakeled stops+1, false schakeld stops
		if (port == false) {
			ad = ((ad * 2) - 1) - 1;
		}
		else { //port =true
			ad = ((ad * 2)) - 1;
		}
		if (ad < aantalStops) {
			stops_rq = ad;
			ET_rq();
			DSP_exe(12);
		}
		break;

	case 3://Draai15 mode

		break;
	}
}
void start() {
	PORTD &= ~(1 << 4); //free lock
	if (GPIOR0 & (1 << 3)) { //motor aan
		OCR2A = Vhome;
		if (~GPIOR0 & (1 << 0)) { //Not while going Home
			if (PRG_fase == 0) OCR2A = Vmin; //versnellen/vertragen instelling er nog ij zetten
			GPIOR0 |= (1 << 4); //versnellen	
			//bereken afremmoment POS_acc is afremafstand totaal, POS_calc is afrempositie
			//prescaler 64;128;256;1024 (100 101 110 111) bit 3 altijd true? nog in EEPROM MEM
			//aantal steps berekenen
			POS_acc = 0;
			for (byte i = Vmax; i <= Vmin; i++) {
				//steptijd/prescaler * clk(0.000.000.00625*i = aantal steps op deze snelheid
				//accStep duur van 1 snelheidstap in milisec from MEM_read()
				POS_acc = POS_acc + (accStep * 1000) / (puls*i);
			}
			//richting en POS_calc instellen
			if (GPIOR0 & (1 << 1)) { //up
				if ((POS_rq - POS) / 2 <= POS_acc) {
					//Serial.println("kleiner");
					POS_calc = POS_rq - ((POS_rq - POS) / 2); //halverwege tussen POS en POS_rq
				}
				else {
					POS_calc = POS_rq - POS_acc;
				}
			}
			else { //down
				if ((POS - POS_rq) / 2 <= POS_acc) {
					//Serial.println("groter");
					POS_calc = POS_rq + ((POS - POS_rq) / 2);
				}
				else {
					POS_calc = POS_rq + POS_acc;
				}
			}
			//Serial.print(F("POS_acc: "));  Serial.println(POS_acc);
			//Serial.print("POS: "); Serial.println(POS);
			//Serial.print("POS_rq: "); Serial.println(POS_rq);
			//Serial.print("POS_calc: "); Serial.println(POS_calc);
		}


		//start motor
		TCCR2B |= (1 << 3);
		TIMSK2 |= (1 << 1); //enable interupt 
		GPIOR0 |= (1 << 5);
		PORTB |= (1 << 0);//Run led
		EIMSK &= ~(1 << INT0); //disable DCC receive
	}
}
void stop() {
	TCCR2B &= ~(1 << 3);
	TIMSK2 &= ~(1 << 1);
	GPIOR0 &= ~(1 << 4);
	GPIOR0 &= ~(1 << 5);
	PORTB &= ~(1 << 0); //Run led 
	EIMSK |= (1 << INT0); //enable DCC receive


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
	PRG_fase = 0;
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
		if (POS == 0) {
			stop();
			//Serial.println("nul");
		}
		else if (POS == POS_rq) {
			stop();
			PORTD |= (1 << 4); //lock bridge on
			//Serial.println("stop");
		}
		else if (POS == POS_calc) {
			//if (PRG_fase == 0) { //alleen bij in bedrijf, instelling nog erbij		
			GPIOR1 |= (1 << 1); //call SLOW() via loop
		}
	}
}
void FAST() { //versnellen tijd gebaseerd
	if (millis() - accTijd > accStep) {
		accTijd = millis();
		OCR2A--; //verhoog snelheid
		if (OCR2A == Vmax)GPIOR0 &= ~(1 << 4); //stop versnellen
	}
}
void SLOW() { //vertragen positie en tijd gebaseerd, called from loop 
	GPIOR1 &= ~(1 << 1); //reset bit thats calls SLOW()
	GPIOR0 &= ~(1 << 4); //reset versnelling
//aantal stappen berekenen met deze snelheid. 
	if (OCR2A < Vmin) OCR2A++;
	POS_acc = (accStep * 1000) / (puls*OCR2A);
	//richting
	if (GPIOR0 & (1 << 1)) { //going up
		POS_calc = POS + POS_acc;
	}
	else { ///going down
		POS_calc = POS - POS_acc;
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
	//snelheden worden als 255-Vsnelheid getoond
	Vhome = EEPROM.read(110);
	if (Vhome == 0xFF)Vhome = 20;
	Vmin = EEPROM.read(111);
	if (Vmin == 0xFF)Vmin = 100;
	Vmax = EEPROM.read(112);
	if (Vmax == 0xFF)Vmax = 5; //Treinlift 4
	Vstep = EEPROM.read(113);
	if (Vstep > 3) Vstep = 1; //TurN 3, Treinlift 1

	aantalStops = EEPROM.read(114);
	if (aantalStops > 16)aantalStops = 8;

	Vaccel = EEPROM.read(115);
	if (Vaccel > 10)Vaccel = 3;

	DCC_adres = EEPROM.read(103);
	if (DCC_adres == 0xFF)DCC_adres = 1;
	DCC_mode = EEPROM.read(104);
	if (DCC_mode > 10)DCC_mode = 1;

	for (byte i = 0; i < 16; i++) {
		EEPROM.get(0 + (5 * i), stops[i]);
		//instelling treinlift
		if (stops[i] == 0xFFFFFFFF) {

			//stops[i] = 500 + i * 500; //draaischijf

			//treinlift
			if (i == 0) {
				stops[0] = 10000;
			}
			else {
				stops[i] = 400000 * i + 10000;
			}
		}
	}
	stops_status = EEPROM.read(100);
	if (stops_status == 0xFF)stops_status = 0;
	COM_V();
}
void COM_V() { //diverse berekeningen voor snelheid
	TCCR2B |= (Vstep << 0); //prescaler
	switch (Vstep) {
	case 0:
		puls = 64 * 0.0625;
		break;
	case 1:
		puls = 128 * 0.0625;
		break;
	case 2:
		puls = 256 * 0.0625;
		break;
	case 3:
		puls = 1024 * 0.0625;
		break;
	}
	//tijd berekenen van 1 vertraag stap
	accStep = Vaccel * 1000 / (Vmin - Vmax);
}
void DSP_exe(byte txt) {
	EIMSK &= ~(1 << INT0); //interrupt DCC ontvangst ff uit
	cd;
	byte et;
	et = stops_rq + 1;
	switch (txt) {
	case 10:
		regelst; display.print(F("going home....."));
		break;
	case 12:
		regelb; display.print(et);
		if (~GPIOR0 & (1 << 3)) { //motor uit
			regelst; display.print("Stop");
		}
		break;
	case 15://keuze in te stellen stops
		regel1s; display.print(F("stops instellen "));
		regel2; display.print(et);
		break;
	case 16: //instellen stops
		regel1s; display.print(F("Instellen stops ")); display.print(et);
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
		regel1s; display.print(F("Handmatig, positie"));
		regel2; display.print(POS);
		break;
	case 40:
		regel1s; display.print(F("Instellingen"));
		switch (PRG_level) {
		case 0:
			regel2;
			display.print(F("Vhome: ")); display.print(255 - Vhome);
			break;
		case 1:
			regel2;
			display.print(F("Vmin: ")); display.print(255 - Vmin);
			break;
		case 2:
			regel2;
			display.print(F("Vmax: ")); display.print(255 - Vmax);
			break;
		case 3:
			regel2;
			display.print(F("Vstep: ")); display.print(Vstep + 1);
			break;
		case 4:
			regel2;
			display.print(F("Vaccel: ")); display.print(Vaccel);
			break;

		case 5:
			regel2;
			display.print(F("Stops: ")); display.print(aantalStops);
			break;
		case 6:
			display.print(F(" DCC: ")); regel2;
			switch (DCC_mode) {
			case 0:
				display.print(F("Single"));
				break;
			case 1:
				display.print(F("Duo-RA"));
				break;
			case 2:
				display.print(F("Duo-AR"));
				break;
			case 3:
				display.print(F("Draai15"));
				break;
			}
			break;
		}

		break;
	case 50:
		regel1s; display.print(F("Modes "));
		switch (PRG_level) {
		case 0:
			display.print(""); display.print(F("Geen functie"));
			break;
		}
		break;
	case 60:
		regel1s; display.println("DCC adres");
		regel2; display.print(DCC_adres * 4 - 3); display.print("("); display.print(DCC_adres); display.print("-1)");
		break;
	}

	display.fillRect(80, 50, 128, 64, BLACK);

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
		case 0: //kiezen stops
			DSP_exe(15);
			break;
		case 1: //instellen gekozen stops
			DSP_exe(16);
			break;
		case 2: //max levels
			PRG_level = 0;
			DSP_exe(15);
			stops[stops_rq] = POS;
			stops_status &= ~(1 << stops_rq);
			EEPROM.put(0 + (5 * stops_rq), POS);
			EEPROM.update(100, stops_status);
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
			SW_encoder(true);
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
			SW_encoder(false);
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
			stops_rq = 0; //move to stop 0
			POS_rq = stops[0];
			if (POS_rq > 0) {
				up; start();
			}
			DSP_exe(12);//DSP_prg();
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
		case 3: //Instellingen met byte waardes
			PRG_level++;

			if (PRG_level > 6)PRG_level = 0;
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
		case 2: //instellen stops
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
		EEPROM.update(113, Vstep);
		EEPROM.update(114, aantalStops);
		EEPROM.update(115, Vaccel);
		EEPROM.update(104, DCC_mode);
		COM_V();
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
	PORTD &= ~(1 << 4); //free lock at new position request
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
		POS_rq = stops[stops_rq];
		if (POS < POS_rq) {
			up;
		}
		else {
			down;
		}
		if (POS != POS_rq) {
			start();
		}
		else {
			PORTD |= (1 << 4); //vrij geven
		}
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
	case 2: //stops instellen
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
	case 3: //waardes instellingen
		if (dir) {
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
			case 3:
				Vstep--;
				if (Vstep > 3) Vstep = 3;
				break;
			case 4: //Vaccel
				Vaccel--;
				if (Vaccel > 10)Vaccel = 10;
				break;
			case 5: //aantal stops
				aantalStops--;
				if (aantalStops < 1) aantalStops = 16;
				break;
			case 6: //DCC modi
				DCC_mode--;
				if (DCC_mode > 3)DCC_mode = 3;
				break;
			}
		}
		else {
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
			case 3:
				Vstep++;
				if (Vstep > 3)Vstep = 0;
				break;
			case 4: //Vaccel
				Vaccel++;
				if (Vaccel > 10)Vaccel = 0;
				break;
			case 5: //
				aantalStops++;
				if (aantalStops > 16)aantalStops = 1;
				break;
			case 6: //DCC modi
				DCC_mode++;
				if (DCC_mode > 3)DCC_mode = 0;
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
		stops_rq++;
		if (stops_rq > aantalStops - 1)stops_rq = 0;
	}
	else {
		stops_rq--;
		if (stops_rq > aantalStops - 1)stops_rq = aantalStops - 1;
	}
}
void loop() {
	//tbv dekoder
	DEK_DCCh();
	slowcount++;
	if (slowcount == 0XFF) {

		if ((GPIOR0 & (1 << 5)) && (GPIOR0 & (1 << 4)))FAST(); //versnellen
		if (GPIOR1 & (1 << 1))SLOW(); //vertragen

		count++;
		if (count == 0) {
			if (GPIOR0 & (1 << 6))RUN_rq();
			if (COM_reg & (1 << 0))ET_rq();
		}
		SW_read();
	}
}
