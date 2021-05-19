/*
 Name:		TurN.ino
 Created:	31jan2021
 Author:	rob Antonisse

Sketch voor Arduino uno , algemeen toepasbaar voor et maken van een draaischijf, rolbrug of treinenmagazijn.
Voorziet in:
Aandrijving en aansturing van een NEMA stappenmotor
DCC ontvangst en besturing
Mogelijkheid voor aansturen door treinbesturingsprogrammaas als Koploper en Itrain

Uitgegaan van treinlift project op 31jan2021

Versie V3-1 bedoeld voor draaischijf zonder adres positionering.

25april2021 begonnen aan versie V3-2 algemene versie
Algemeen:
Homen. Voor draaischijf altijd tegen de klok in. Rolbrug naar voren. Lift naar beneden. Algemene draairichting met hardware aanpassen hieraan.
Aampassingen:
Instelling keuze toepassing 0=draaischijf, 1=rolbrug, 2=lift

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

//V3.00 Aantal buffers va 6 > 4 gebracht om geheugen ruimte te sparen. NIET GECHECKED! 

volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts
//boolean DEK_Monitor = false; //shows DCC commands as bytes
byte DEK_Reg; //register voor de decoder 
byte DEK_Status = 0;
byte DEK_byteRX[4]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte DEK_countPA = 0; //counter for preample
byte DEK_BufReg[4]; //registerbyte for 12 command buffers, can be lowered.
byte DEK_Buf0[4];
byte DEK_Buf1[4];
byte DEK_Buf2[4];
byte DEK_Buf3[4];
byte DEK_Buf4[4];
byte DEK_Buf5[4];
//**End declaration for deKoder

byte DCC_adres;
byte DCC_mode; //EEPROM 
byte COM_reg;
byte MEM_reg;
byte slowcount;

long POS;
long POS_rq;
long POS_acc; //aantal stappen tussen afrem posities
long POS_calc; //afrem positie

//unsigned long stops[16]; //v3.0 unsigned

struct stop
{
	long pos; //(melders)Afstand naar Volgende stop. (Home)stopplek
	int width; //(melders)afstand tussen de twee melder edges
	int fine; //afwijking middenount tussen edges
	byte reg; //register
};
//bit0=stop ingesteld(false) stop niet ingesteld(true)

struct stop stops[16]; //melder met adres 1 is stops[0]

//byte setcount;
//byte setstop; //welke stop is in het proces van bepalen, setten
byte aantalStops;
byte toepassing;
byte lastup = 0;
byte lastdown = 0;
byte stops_rq;
byte stops_current;//V3.00 holds the current stop
byte switchstatus[3];
byte switchcount;
byte melderadres;
byte melderTemp;
byte melderOld;
byte PRG_fase;
byte PRG_level;
byte ENC_count;
byte Vhome;
byte Vmin;
byte Vmax;
byte Vstep; //instelling prescaler 64;128;259;1024
byte count;
byte startcount;

unsigned long runwait;
byte Vaccel = 3; //ingestelde acceleratietijd in seconden
float accStep; //berekende tijd per snelheids step in microsec
float puls;
unsigned long accTijd; //verstreken tijd
byte runfase;

//temps


void setup() {
	Serial.begin(9600);
	//clear hardware registers 
	GPIOR0 = 0;
	GPIOR1 = 0;
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	/*
	Dit display is 128x64 pixels. Een pixel kan aan of uit zijn.
	Totaal dus 8192 pixels.
	Verdeeld over 1024 bytes.
	Gebruik van deze library/display bezet dus 1024bytes is helft van het geheugen van de arduino.
	Overhead van de arduino is ongeveer een 10% van het geheugen, dus kan maar 40% van het memory worden gebruikt voor de rest. Ongeveer 850bytes.
	Overloop van het geheugen veroorzaakt enorm vreemde onvoorspeelbare  fouten.
	*/

	//**begin Setup for DeKoder 
	//Serial.begin(9600);
	//interrupt on PIN2
	DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA � External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//External Interrupt Mask Register bit0 INT0 > 1
	//**End Setup for DeKoder


	cd; //clear display
	DSP_settxt(10, 10, 1); display.print(F("www.wisselmotor.nl"));
	DSP_settxt(10, 30, 2); display.print("TurN V3.0");
	display.display();
	delay(2000);

	//ports
	PORTC |= (15 << 0); //pullup  A0~A3

	DDRB |= (15 << 0); //Pin8~Pin11 as outputs
	DDRD |= (1 << 4); //pin4 as output (groene led) lock

	//PINB |= (1 << 0); //set pin8 V3.00 uit waarom moet dit hier...?
	//DDRD |= (B11110000 << 0); //pin 7,6,5,4 output
	//factory reset

	DDRD |= (1 << 7); //set pins om switches uit te lezen voor de factory reset 
	PORTD &= ~(1 << 7);
	//Serial.println(PINC);
	if (PINC == 54) {
		cd;
		DSP_settxt(10, 15, 2); display.print(F("Factory"));
		DSP_settxt(10, 50, 1); display.print(F("Druk reset..."));
		display.display();
		delay(500);
		FACTORY();
		delay(500);
	}
	DDRD &= ~(B11100000); //Pins 7,6,5 as input
	PORTD |= (B11110000); //pins 7,6,5 set pull-up

	//timer 2 settings on pin11 compare match
	TCCR2A |= (1 << 6); //toggle pin6
	TCCR2A |= (1 << 1);
	TCCR2B |= (1 << 2); //prescaler always 4  on(default true???)
	OCR2A = 255; //snelheid
	//init

	switchstatus[0] = 0xFF;
	switchstatus[1] = 0xFF;
	switchstatus[2] = 0xFF;


	switchcount = 2; //zorgt ervoor dat eerst switchstatus 0 wordt getest, noodzakelijk, anders opstartproblemen
	MEM_read();
	//nodig voor opstart procedure
	if (MEM_reg & (1 << 0)) { //melder mode eerst 1x melderadres bepalen	
		GPIOR1 |= (1 << 2); //motor pas nadat er een change in de melderstatus is geweest.
		//SW_read(); //V3.0 bepaal of een van de stops actief is, lees schakelaars
	}
	else { //home mode direct naar home melder zoeken
		MOTOR();
	}

	//MOTOR();
	//RUN_home(); V3.0 also called in Motor
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
	//Serial.print(F("Start POS= ")); Serial.println(POS);
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
		} //going home, searching stop
	//start motor
		startmotor();

	}
}
void startmotor() {
	TCCR2B |= (1 << 3);
	TIMSK2 |= (1 << 1); //enable interupt 
	GPIOR0 |= (1 << 5);
	PORTB |= (1 << 0);//Run led rood
	EIMSK &= ~(1 << INT0); //disable DCC receive
}
void stop() {
	//Serial.print(F("stop. POS= ")); Serial.println(POS);
	TCCR2B &= ~(1 << 3);
	TIMSK2 &= ~(1 << 1);
	GPIOR0 &= ~(1 << 4);
	GPIOR0 &= ~(1 << 5);
	PORTB &= ~(1 << 0); //Run led 
	EIMSK |= (1 << INT0); //enable DCC receive

}
void MOTOR() {
	//Serial.println(F("Motor"));
	GPIOR0 ^= (1 << 3);
	if (~GPIOR0 & (1 << 3)) {
		PORTB |= (1 << 1);
		stop();
		DSP_exe(20);
	}
	else {
		PORTB &= ~(1 << 1);
		//na noodstop moet alle waardes worden gereset in melder-mode
		//NIET in home-mode want die waardes zijn handmatig ingevoerd
		//tis voldoende de .reg bytes te resetten.
		if (MEM_reg & (1 << 0)) {//melder-mode
			for (byte i = 0; i < 16; i++) {
				stops[i].reg = 0xFF;
			}
			stops_current = 0;
			stops_rq = 0;
			runfase = 0;
			if (melderadres > 0) GPIOR1 |= (1 << 4); //niet vrij geven als noodstop in melder
		}
		RUN_home();
	}
	PRG_fase = 0;
}

void RUN_home() { //move to HOME	//Serial.println(melderadres);
	//Serial.println(F("Runhome"));
	//Serial.println(F("run_home"));
//Serial.println(melderadres);
	if (MEM_reg & (1 << 0) && melderadres > 0) { //Melder-mode	

		//Als brug in een stop staat niks doen in melder-mode
		stops_rq = melderadres - 1;
		DSP_exe(12);
		stop();
		lock(); //hmmm V3.00 denk ik niet hier....
		stops_current = stops_rq;
		//POS = 0;
	}
	else { //home-mode of op tussengebied, geen active melder	

		down;
		GPIOR0 |= (1 << 0); //home zoeken
		start();
		DSP_exe(10);
	}
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
		if (POS == 0 && ~MEM_reg & (1 << 0)) { //only in home-mode
			stop();
		}
		else if (POS == POS_rq) {

			if (MEM_reg & (1 << 0)) {//melder mode
				switch (runfase) {
				case 10:
					//Serial.println(F("stop in ISR"));
					stop();
					lock();
					break;

				case 20:
					if (melderadres == 0) { //bereikt voor stop edge melder, dus te kort gedraait
						runfase = 30;
						//Serial.println(F("pos bereikt"));

					}
					else { //bereikt NA stopedge melder, dus te ver gedraait
						Serial.println(F("Noodstop, te ver gedraait"));
						stop();
					}

					break;
				}
			}
			else { //home mode
				stop();
				PORTD |= (1 << 4); //lock bridge on
			}

		}
		else if (POS == POS_calc) {
			if (MEM_reg & (1 << 0 && (runfase > 0))) {
				GPIOR1 |= (1 << 1); //call SLOW() via loop altij in home-mode						
			}
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
	//Serial.println(F("factory"));
	for (byte i = 0; i < 500; i++) {
		EEPROM.update(i, 0xFF);
	}
}
void MEM_read() {
	//instellingen voor een draaischijf zonder vertraging
	//snelheden worden als 255-Vsnelheid getoond

	MEM_reg = EEPROM.read(102);

	Vhome = EEPROM.read(110);
	if (Vhome == 0xFF)Vhome = 50;
	Vmin = EEPROM.read(111);
	if (Vmin == 0xFF)Vmin = 150;
	Vmax = EEPROM.read(112);
	if (Vmax == 0xFF)Vmax = 20; //Treinlift 4
	Vstep = EEPROM.read(113);
	if (Vstep > 3) Vstep = 3; //TurN 3, Treinlift 1

	aantalStops = EEPROM.read(114);
	if (aantalStops > 16)aantalStops = 4; //testen V3.0 moet zijn 8

	Vaccel = EEPROM.read(115);
	if (Vaccel > 10)Vaccel = 8;

	toepassing = EEPROM.read(116); //0==draaischijf, 1=rolbrug 2=lift
	if (toepassing > 2)toepassing = 0;

	DCC_adres = EEPROM.read(103);
	if (DCC_adres == 0xFF)DCC_adres = 1;
	DCC_mode = EEPROM.read(104);
	if (DCC_mode > 10)DCC_mode = 1;
	//terug laden postities 
	for (byte i = 0; i < 16; i++) {
		EEPROM.get(200 + (5 * i), stops[i].pos);
		EEPROM.get(300 + (5 + 1), stops[i].width);
		EEPROM.get(400 + (5 + 1), stops[i].fine);
		stops[i].reg = EEPROM.read(i + 1); //reg = register met 8 booleans
		//bit0 posities bepaald

		if (MEM_reg & (1 << 0)) { //melder-mode

		}
		else { //home-mode
			if (stops[i].pos == 0xFFFFFFFF) {
				stops[i].pos = (900 * i) + 500; //draaischijf			
			}
		}
	}
	//stops_status = EEPROM.read(100);
	//if (stops_status == 0xFF)stops_status = 0;
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
		regelst; display.print(F("Positie zoeken...."));
		break;
		//case 11:
		//	regelst; display.print(F("Searching stop..."));
		//	break;
	case 12: //In bedrijf aanduiding
		regelb; display.print(et);
		if (~GPIOR0 & (1 << 3)) { //motor uit
			regelst; display.print(F("Stop"));
		}
		if (melderadres > 0) {
			DSP_settxt(104, 5, 2);
			//if (melderadres < 10)display.print(0);
			//display.print("1");
			display.print(melderadres);
			//display.drawCircle(110, 15, 15, WHITE);
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
		case 6: //toepassing instellen
			regel2;
			switch (toepassing) {
			case 0:
				display.print(F("Schijf"));
				break;
			case 1:
				display.print(F("Rolbrug"));
				break;
			case 2:
				display.print(F("Lift"));
				break;
			}
			break;

		case 7:
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
			display.print(""); display.print(F("Posities met..")); regel2;

			if (MEM_reg & (1 << PRG_level)) {
				display.print(F("Melders"));
			}
			else {
				display.print(F("Home"));
			}
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
			stops[stops_rq].pos = POS;
			//stops_status &= ~(1 << stops_rq);
			EEPROM.put(200 + (5 * stops_rq), POS);
			//EEPROM.update(100, stops_status);
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
void SW_read() { //lezen van schakelaars, called from loop and setup, before motorstart
	byte sr; byte changed; byte v;
	GPIOR1 ^= (1 << 5);
	if (~GPIOR1 & (1 << 5)) { //instellen GPIOR1
		switchcount++;
		if (switchcount > 2)switchcount = 0;
		//DDRD &= ~(B11100000); //set H-z pin 7,6,5

		switch (switchcount) {
		case 0:
			DDRD &= ~(1 << 5);
			DDRD &= ~(1 << 6);
			DDRD |= (1 << 7);
			PORTD &= ~(1 << 7); //set pin			
			break;
		case 1:
			DDRD &= ~(1 << 5);
			DDRD &= ~(1 << 7);

			DDRD |= (1 << 6);
			PORTD &= ~(1 << 6); //set pin	break;	
			break;
		case 2:
			DDRD &= ~(1 << 6);
			DDRD &= ~(1 << 7);

			DDRD |= (1 << 5);
			PORTD &= ~(1 << 5); //set pin
			break;
		}
	} //GPIOR1
	else { //lezen GPIOR1	
		//Dit werkt alleen met pullups naar 5V van 1K (interne pullup is te zwak)
		/*
		Schakelaars werken als volgt.
		pinnen 7,6,5 in de setup worden ze gezet als input met pull up. Pinnen zijn dan hi-Z
		achtereenvolgend wordt 1 van de pinnen als output gezet en laag.
		Doordat de twee andere pinnen Hi-z staan kan er geen 'sluiting' ontstaan.
		De C-port wordt nu gelezen en bevat de actuele status van de schakelaars overeenkomend
		met de als output gemaakte pin.
		Verder is er een issue met de snelheid, te traag zal de encoder niet lekker laten werken.
		En een issue met de opstart situatie, switchcount 0 moet als eerste woren gelezen. In setup is daaom switchcount
		op 2 gezet.

		na V3.00
		boolean GPIOR1 bit5 maakt de read cyslus in twee delen, Eerste doorloop worden pinnen NAAR de schakelaar alle
		Hoog Z gezet. In de tweede doorloop wordt dan de C poort gelezen.
		19mei2021 schjnt nu goed te werken, in 1 doorloop waren telkens wisselende fouten bij opstarten


		*/
		//	sr = PINC;
		sr = PINC << 4;
		sr = sr >> 4;
		//	sr |=(B11110000);

		changed = sr ^ switchstatus[switchcount];
		//if (changed > 0) Serial.println("ch");
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

		if (MEM_reg & (1 << 0)) { //als positie met melders
			//if (GPIOR1 & (1 << 2)) SW_melderadres();
			if (changed > 0 && switchcount == 1)SW_melderadres(); //melders veranderd
		}

	} //GPIOR1
}

void SW_melderadres() { //called from sw_read if MEM_reg bit 0 is true (mode-melders) and melderstatus changed
	melderadres = 0;
	//Serial.print(F("SW_melderadres, switchstatus: ")); Serial.println(switchstatus[1], BIN);// zijn de melders
	for (byte i = 0; i < 4; i++) {
		if (~switchstatus[1] & (1 << i)) {
			switch (i) {
			case 0:
				melderadres = melderadres + 1;
				break;
			case 1:
				melderadres = melderadres + 2;
				break;
			case 2:
				melderadres = melderadres + 4;
				break;
			case 3:
				melderadres = melderadres + 8;
				break;
			}
		}
	}

	if (GPIOR1 & (1 << 2)) { //set in setup
		GPIOR1 &= ~(1 << 2);
		MOTOR();
	}


	if ((melderTemp > 0 && melderadres == 0) || (melderTemp == 0 && melderadres > 0)) {
		//dit dient om 'tussenmelderadres' onmogelijk te maken van 3 via 2 naar 0 bv...
		//Serial.print(F("Melderadres: ")); Serial.println(melderadres);
		//hier VERANDERING van melder status
		MA_changed(); //Als bit3 =true melders uitschakelen, draaien naar POS_rq en stoppen. 
		DSP_exe(12);
		melderOld = melderadres;
	}
	melderTemp = melderadres;
}

void MA_changed() {
	//Serial.print(F("adres: ")); Serial.println(melderadres);
	byte stp;

	switch (runfase) {
	case 0: //zoeken naar stops
		if (melderadres == 0) { //melder vrijgekomen, breedte melder gemeten
			stp = melderOld - 1; //stop=melder-1
			if (stp != stops_current) {
				F_width(stp); //Niet bij stop van vertrek, overstaande melder van deze stop moet gepasseerd zijn
			}
			else {
				POS = 0;
			}
		}
		else { //melderadres > 0; stop 'melderadres-1' bereikt, afstand gemeten	
			if (GPIOR0 & (1 << 1)) { //richting. Draait up, downmelder bereikt
				stp = melderadres - 2; //afstand voorliggende stop zijn bepaald, dus -2
			}
			else { //draait down, upmelder bereikt
				stp = melderadres - 1; //afstand tot deze stop is bepaald dus -1

				if (GPIOR0 & (1 << 0)) { //zoekt positie
					//Serial.println(F("MA-changed found stop"));
					if (melderadres == aantalStops) { //voorbij eerste stop gestart
						stops_current = stp;
						up;
						stops_rq = 0;
					}
					else {
						if (stp == 0)stops_current = 1;

						stops_rq = stp;
					}
					GPIOR0 &= ~(1 << 0); //stop zoeken
					POS = 0;
				}
			}
			F_pos(stp);
			POS = 0;
		}
		break;
	case 30:
		//gedraait voor melder, melder nu bereikt. 
		//stops_rq moet bekend zijn...
		POS_rq = stops[stops_rq].width / 2;
		if (GPIOR0 & (1 << 1)) {//updraaien
			POS = 0;
		}
		else {//down draaien
			POS = POS_rq;
			POS_rq = 0;
		}
		RUN_rq();
		runfase = 10;
		break;
	}
}
void F_width(byte stp) { //breedte van een melder bepaald
	//Serial.println(F("F_width"));
	stops[stp].reg &= ~(1 << 1);
	stops[stp].width = abs(POS); //altijd een positieve waarde
	if (stp == stops_rq) {
		stop();
		//hier is van de stops_rq melder de width bekend

		if (POS > 0) {
			POS_rq = POS - stops[stp].width / 2;
		}
		else {
			POS_rq = POS + stops[stp].width / 2;
		}

		RUN_rq();
		runfase = 10;
		//monitor();
	}

	else {
		POS = 0;//reset position voor volgende meting
	}
}
void F_pos(byte stp) {
	stops[stp].reg &= ~(1 << 2);
	stops[stp].pos = abs(POS); // Altijd positief waarde
}
void monitor() {
	//geeft debug data
	for (byte i = 0; i < aantalStops; i++) {
		Serial.print(F("Stop:")); Serial.print(i); Serial.print(F("  POS= ")); Serial.print(stops[i].pos);
		Serial.print(F("  width= ")); Serial.print(stops[i].width); Serial.print(F("  reg: ")); Serial.println(stops[i].reg, BIN);
	}
	Serial.println("");
}
void runcenter(byte stop) {
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
	//Serial.print("Switch on: "); Serial.println(sw);
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
		//MELDERS ******
	case 4:
		if (~MEM_reg & (1 << 0)) { //als positie met home switch 
			//Serial.println(F("melder0"));
			//niet voor testen

			if (GPIOR0 & (1 << 0)) { //home gevonden
				stop();
				POS = 0;
				GPIOR0 &= ~(1 << 0);
				stops_rq = 0; //move to stop 0
				POS_rq = stops[0].pos;
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
			//dient alleen tjdens bouwen om de motor te kunnen laten draaien.
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
			if (PRG_level > 7)PRG_level = 0;
			DSP_exe(40);
			break;

		case 4: //modes (MEM_reg instellingen)
			PRG_level++;
			if (PRG_level > 0)PRG_level = 0;
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
		case 3: //instellingen
			SW_encoder(false);

			break;
		case 4: //modes boolean MEM_reg 0=positie met 			
			SW_encoder(false); //parameter nu niet relevant, boolean altijd toggle
			break;

		case 5:
			SW_encoder(false);
			break;
		}
	}
	else { //switch release
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
		MOTOR(); //toggled motor aan of uit. Ook vanuit setup...eenmalig 
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
		EEPROM.update(116, toepassing);
		COM_V();
		OCR2A = Vhome;
		PRG_fase = 0;
		DSP_prg();
		break;
	case 4://diverse	
		EEPROM.update(102, MEM_reg);
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
	if (~GPIOR0 & (1 << 5)) { //motor draait niet
	//tijdens positie testen ff uit
		PRG_fase++;
		if (PRG_fase > 5)PRG_fase = 0;
		DSP_prg();
		PRG_level = 0;
	}
}
void ET_rq() {
	PORTD &= ~(1 << 4); //free lock at new position request
	if (~COM_reg & (1 << 0)) {
		COM_reg |= ((1 << 0));
		runwait = millis();
	}
	else {
		if (millis() - runwait > 3000) {
			if (MEM_reg & (1 << 0)) { //melders-mode
				if (stops_current != stops_rq)RUN_rq_M();
			}
			else { //home-mode
				POS_rq = stops[stops_rq].pos;
				RUN_rq();
			}
			COM_reg &= ~(1 << 0);
		}
	}
}
void RUN_rq() {
	if (GPIOR0 & (1 << 5)) { //motor draait
		if (~GPIOR0 & (1 << 6)) { //iets met afremmen?
			GPIOR0 |= (1 << 6);
		}
	}
	else {
		GPIOR0 &= ~(1 << 6);
		if (POS < POS_rq) {
			up;
		}
		else {
			down;
		}
		if (POS != POS_rq) { //als al in request stop, dan niet draaien en vrij geven
			start();
		}
		else {
			PORTD |= (1 << 4); //vrij geven
		}
	} //	if (GPIOR0 & (1 << 5)) { //motor draait
}
void RUN_rq_M() { //called from et_rq (stops_rq==stopscurrent komt niet voor)
	boolean bekend = true;
	runfase = 0;
	//stops_rq=waar die naar toe moet, stop_current staat tie in
	POS = 0; //zet huidige positie op 0
	long dist = stops[stops_current].width / 2; //van middenpunt naar melder(.fine moet hier nog bij komen)
	//kijken of route bekend is
	if (stops_rq > stops_current) {
		up; //vreemde plek alleen functioneel als bekend false blijkt te zijn, scheelt een if statement
		for (byte i = stops_current; i < stops_rq; i++) { //alle route delen moeten bekend zijn.
			if (stops[i].reg & (1 << 2))bekend = false; //afstand tussen onderstop naat bovenstop (niet) bekend			
			if (stops[i].reg & (1 << 1))bekend = false; //breedte melder bekend
			if (stops[i + 1].reg & (1 << 1))bekend = false;

			dist = dist + stops[i].pos;
			if (i != stops_current) dist = dist + stops[i].width;
		}
	}
	else { //rq<current
		down; //zie up
		for (byte i = stops_rq; i < stops_current; i++) { //alle route delen moeten bekend zijn.
			if (stops[i].reg & (1 << 2))bekend = false; //afstand tussen onderstop naar bovenstop (niet) bekend
			if (stops[i].reg & (1 << 1))bekend = false; //breedte melder (niet) bekend aankomst stop
			if (stops[i + 1].reg & (1 << 1))bekend = false; //breedte melder bekend vertrek stop

			dist = dist + stops[i].pos;
			if (i != stops_rq) dist = dist + stops[i].width;
		}
	}

	if (bekend) {
		runfase = 20;
		//Serial.print(F("dist=")); Serial.println(dist);
		//GPIOR1 |= (1 << 3); //melders uitschakelen, tempie

		if (GPIOR0 & (1 << 1)) { //richting up
			POS = 0;
			POS_rq = dist;
		}
		else { //richting down
			POS_rq = 0;
			POS = dist;
		}
		RUN_rq();
	}
	else { //niet bekend
		runfase = 0;
		//Serial.println(F("run_rq niet bekend"));
		OCR2A = Vhome;
		startmotor();
	}
}
void SW_encoder(boolean dir) {
	switch (PRG_fase) {
	case 0:
		if (~GPIOR0 & (1 << 5)) { //als motor draait keuze nieuwe stop niet mogelijk
			ENC_select(dir);
			ET_rq();
			DSP_exe(12);
		}
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
			case 6: //toepassingen
				toepassing--;
				if (toepassing > 2)toepassing = 2;
				break;
			case 7: //DCC modi
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
			case 6://Toepassing
				toepassing++;
				if (toepassing > 2)toepassing = 0;
				break;
			case 7: //DCC modi
				DCC_mode++;
				if (DCC_mode > 3)DCC_mode = 0;
				break;
			}
		}
		DSP_exe(40);
		break;
	case 4: //mode instellen MEM_reg
		MEM_reg ^= (1 << PRG_level); //0=positie met
		DSP_exe(50);
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
void lock() {
	//doet alles om de brug te vergrendelen en vrij te geven vor next gebruik
	//voorlopig alleen groene ledje op PIN4
	//issue na noodstop BINNEN een melder mag brug niet vrij worden gegeven
	//eventueel automatisch verplaatsen? kan hier ook maar voorlopig alleen led niet vrijgeven
	stops_current = melderadres - 1;
	if (~GPIOR1 & (1 << 4)) PORTD |= (1 << 4); //lock bridge on, NOT after noodstop
	GPIOR1 &= ~(1 << 4);
	runfase = 0;
}
void free() {
	//Maakt de brug vrij om te kunnen draaien
	PORTD &= ~(1 << 4); //groene led, lock low
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
	
	DEK_DCCh();//tbv dekoder
	slowcount++;//Counter voor langzame processen 1xin 255 cycli
	if (slowcount == 0) { //V3.00 is deze tijd 2x zo lang geworden, check of encoder het nog lekker doet
		SW_read();
		if ((GPIOR0 & (1 << 5)) && (GPIOR0 & (1 << 4)))FAST(); //versnellen
		if (GPIOR1 & (1 << 1))SLOW(); //vertragen
		count++;
		if (count == 0) {
			if (GPIOR0 & (1 << 6))RUN_rq();
			if (COM_reg & (1 << 0))ET_rq();
		}
	}
}
