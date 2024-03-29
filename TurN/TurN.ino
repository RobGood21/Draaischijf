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

Versie 4.0
25april2021 begonnen aan versie V4.0 algemene versie
Algemeen:
Homen. Voor draaischijf altijd tegen de klok in. Rolbrug naar voren. Lift naar beneden. Algemene draairichting met hardware aanpassen hieraan.
Aampassingen:
Instelling keuze toepassing 0=draaischijf, 1=rolbrug, 2=lift


Versie V5.1 overgegaan op NmraDcc library als decoder

14 okt 2021 V5.02
Ernstige bug (hopelijk) opgelost.
In SW_melderadres, Melder wordt nu 2x achter elkaar gelezen om bouncen en na elkaar van de adres bits te ondervangen.
23okt2021
bug25okt het hoogzetten van pin 4 portd4 doen ook na noodstop bit4 van gpior1 komt daarmee weer als
niet in gebruik
bug m26okt COM_reg weggehaald
bug rw26okt runwait als byte spaart misschien 2 bytes, maar kan alleen via een xtra second byte en second counter
bug cm26okt melder lezing naar 2 teruggebracht met een bit in GPIOR2 bit5
bug sbug26okt onduidelijk issue in start, keuzes niet duidelijk komen niet voor

18nov2021 V5.03
Bug18nov doordat de 'lezing van melders" verdubbelt is in versie V5.02 start de motor niet als niet in een station in melderm
mode, dit verholpen op regel 946
bug19nov prglevel naar 0 bij iedere opschakeling van Prgfase
bug20nov parameter in byte waardes omhoog en omlaag mogelijk

*/

#include <EEPROM.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <splash.h>
#include <Adafruit_SSD1306.h>

#include <NmraDcc.h>
NmraDcc  Dcc;

#define version "V5.03"

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


Adafruit_SSD1306 display(128, 64, &Wire, -1); //constructor display

byte DCC_adres;
byte DCC_mode; //EEPROM 
//byte COM_reg; //m26okt
byte MEM_reg;
byte slowcount; //was een byte

//kunnen dit geen ints worden????
long POS;
long POS_rq;
int POS_acc; //aantal stappen tussen afrem posities m26okt was een long 
long POS_calc; //afrem positie

struct stop
{
	//kunnen pos en width geen int of bytes worden? misschien met een berekende factor?
	long pos; //(melders)Afstand naar Volgende stop. (Home)stopplek
	int width; //(melders)afstand tussen de twee melder edges
	int fine; //afwijking middenpunt moet een signed int zijn pos en neg mogelijk
	byte reg; //register
	//bit 1 afstand naar volgende bovenliggende stop gemeten
	//bit 2 width, breedte tussen melder edges gemeten
};
struct stop stops[16]; //melder met adres 1 is stops[0]

byte aantalStops;
//byte lastup = 0; m26okt wordt niet gebruikt
//byte lastdown = 0; m26okt niet in gebruikt
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
//byte startcount; m26okt wordt niet gebruikt
byte oldmelder;// cm26okt byte meldercount = 0; //misschien zijn counters dubbel te gebruiken?
byte speling;//speling of vrije slag in de aandrijving max 255
byte draai15;
byte dr15pos; //current position in draai15
byte dr15ads; //draai15 aantal tegenover liggende sporen (koploper, Itrain)


//byte seconds
unsigned long runwait; //rw6okt runwait as bytex1000
byte Vaccel = 3; //ingestelde acceleratietijd in seconden
float accStep; //berekende tijd per snelheids step in microsec
float puls;
unsigned long accTijd; //verstreken tijd
byte runfase;

void setup() {
	Serial.begin(9600);
	//clear hardware registers 
	GPIOR0 = 0x00;
	GPIOR1 = 0x00;
	GPIOR2 = 0x00;

	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	/*
	Dit display is 128x64 pixels. Een pixel kan aan of uit zijn.
	Totaal dus 8192 pixels.
	Verdeeld over 1024 bytes.
	Gebruik van deze library/display bezet dus 1024bytes is helft van het geheugen van de arduino.
	Overhead van de arduino is ongeveer een 10% van het geheugen, dus kan maar 40% van het memory worden gebruikt voor de rest. Ongeveer 850bytes.
	Overloop van het geheugen veroorzaakt enorm vreemde onvoorspeelbare  fouten.
	*/
	cd; //clear display
	DSP_settxt(8, 10, 1); display.print(F("www.wisselmotor.nl"));
	DSP_settxt(8, 30, 2); display.print(F("TurN ")); display.print(version);
	display.display();
	delay(2000);

	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder adres

	//ports
	PORTC |= (15 << 0); //pullup  A0~A3
	DDRB |= (15 << 0); //Pin8~Pin11 as outputs


	//factory reset
	DDRD |= (1 << 7); //set pins om switches uit te lezen voor de factory reset 
	PORTD &= ~(1 << 7);
	if (PINC == 54) {
		cd;
		DSP_settxt(10, 15, 2); display.print(F("Factory"));
		DSP_settxt(10, 50, 1); display.print(F("Druk reset..."));
		display.display();
		delay(500);
		FACTORY();
		delay(500);
	}

	DDRD |= (1 << 4); //pin4 as output (groene led) lock

		//moet dit geen outputs zijn?????, nee in prg worden de pins dir en waarde telkens geschakeld
	DDRD &= ~(B11100000); //Pins 7,6,5 as input

	PORTD |= (B11100000); //pins 7,6,5 set pull-up, pin 4 laag

	//timer 2 settings on pin11 compare match
	TCCR2A |= (1 << 6); //toggle pin6
	TCCR2A |= (1 << 1);
	TCCR2B |= (1 << 2); //prescaler always 4  on(default true???)
	OCR2A = 255; //snelheid
	//init

	stops_rq = 0;
	//dr15pos = 1;

	switchstatus[0] = 0xFF;
	switchstatus[1] = 0xFF;
	switchstatus[2] = 0xFF;
	switchcount = 2; //Eerst geteste switchcount nu 0 (switches) 

	MEM_read();

	//nodig voor opstart procedure
	if (MEM_reg & (1 << 0)) { //melder mode eerst 1x melderadres bepalen
		//MOTOR() called from sw_melderadres() nadat 1x demelderadressen zijn getoetst.
		//melderadres dus bekend voordat MOTOR() is called.
		GPIOR1 |= (1 << 2); //flag voor one-shot, anders wordt MOTOR() in iedere loop cycle getest.
	}
	else { //home mode, testen melderadres nu niet nodig.
		MOTOR(true);
	}
}
void notifyDccAccTurnoutBoard(uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower) {
	//Call Back van NmraDcc library
	//called als CV29 bit6 = false decoderadres,channel,poort,onoff (zie setup 'init')
	//interupt mask register cleared in startmotor() en set in stop() om DCC te disabelen als motor draait. ((nog)niet in homen)
	APP_DCC(BoardAddr, OutputPair + 1, Direction, OutputPower);
}
void APP_DCC(int decoder, int channel, boolean port, boolean onoff) {
	//let op deze versie werkt met decoder(adres), channel, port, onoff

	unsigned int ad = 0; byte stop = 0;
	// byte dr15;
	ad = decoder - DCC_adres;
	if (ad >= 0 && ad < 4) { //max 4 decoder adressen, voor single mode 16 stops. 
		ad = ad * 4 + channel;
		switch (DCC_mode) {
		case 0: //Single alleen true port schakeld stops
			if (port == true) {
				stop = ad;
			}
			break;

		case 1: //DUO-RA true schakeld stop false schakeld stop+1
			if (port == true) {
				stop = ad * 2 - 1;
			}
			else { //port =false
				stop = ad * 2;
			}
			break;

		case 2: //DUO-AR  afslaan-rechtdoor
			if (port == false) {
				stop = ad * 2 - 1;
			}
			else { //port =true
				stop = ad * 2;
			}
			break;

		case 3://Draai15 mode
			//dr15 = POS_rq + 1; //huidige stop ophalen BELANGRIJK????
			 //1 decoderadres nodig dus:
			if (decoder == DCC_adres && onoff) {
				//adressen. 
				//reset=1 r(echtdoor) 1=2r 2=2a(fslaan) 4=3r 8=3a
				//start=4r en 4a
				switch (ad) {
				case 1:
					draai15 = 0;
					break;

				case 2:
					if (port) {
						draai15 |= (1 << 0);
					}
					else {
						draai15 |= (1 << 1);
					}
					break;

				case 3:
					if (port) {
						draai15 |= (1 << 2);
					}
					else {
						draai15 |= (1 << 3);
					}
					break;

				case 4:

					if (draai15 == dr15pos) {

						if (GPIOR2 & (1 << 4)) { //flag enable 180

							GPIOR2 ^= (1 << 3); //set direct to counter track
							GPIOR2 &= ~(1 << 4); //disable 180
							stop = dr15();
						}
					}
					else {
						if (port) { //omhoog
							if (draai15 < dr15pos) { //wisseld tussen direct en tegenoverliggend
								GPIOR2 ^= (1 << 3);
							}
						}
						else { //omlaag
							if (draai15 > dr15pos) { //wisseld tusen direct en tegenover
								GPIOR2 ^= (1 << 3);
							}
						}
						stop = dr15();
					}//draai15==dr15pos
					dr15pos = draai15; //huidige positie onthouden
					break; //case 4 van decoder channel
				}
				break; //draai15 mode
			} //decoder==dcc_adres
		}
	}

	//stop request maken
	if (stop > 0 && stops_rq != stop - 1 && stop <= aantalStops && ~GPIOR0 & (1 << 5)) { //alleen als motor uit

		stops_rq = stop - 1;
		ET_rq();
		DSP_exe(12);
	}

}
//**End void's for DeKoder
byte dr15() {
	byte result;
	if (GPIOR2 & (1 << 3)) {
		result = draai15 + dr15ads; //dr15ads is aantal 'overkant' sporen uit koploper/draaischijf tekenen
	}
	else {
		result = draai15;
	}
	return result;
}
void start() {
	//Serial.println(GPIOR0, BIN);
	//PORTD &= ~(1 << 4); //free lock V5.02 weggehaald sbug26okt
	//if (GPIOR0 & (1 << 3)) { //motor aan weggehaald V5.02, only called als motor aan is sbug26okt

	if (~GPIOR0 & (1 << 0)) { //GPIOR0 bit0 true =homing. Positie zoeken
		if (PRG_fase == 0) OCR2A = Vmin; //instellen startsnelheid
		GPIOR0 |= (1 << 4); //versnellen	
		//bereken afremmoment POS_acc is afremafstand totaal, POS_calc is afrempositie
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
				POS_calc = POS_rq - ((POS_rq - POS) / 2); //halverwege tussen POS en POS_rq
			}
			else {
				POS_calc = POS_rq - POS_acc;
			}
		}
		else { //down
			if ((POS - POS_rq) / 2 <= POS_acc) {
				POS_calc = POS_rq + ((POS - POS_rq) / 2);
			}
			else {
				POS_calc = POS_rq + POS_acc;
			}
		}
	}
	else {
		OCR2A = Vhome;
	} //going home, searching stop

	startmotor();
	//} //V5.02  sbug26okt
}
void startmotor() {
	//Serial.println("sm"); //25okt
	//PORTB &= ~(1 << 1); //enable motor, ENA van driver 
	TCCR2B |= (1 << 3); //enable interrupt prescaler 
	TIMSK2 |= (1 << 1); //enable interupt 
	GPIOR0 |= (1 << 5); //flag true motor draait, false motor staat stil
	GPIOR1 |= (1 << 6); //disable switches
	PORTB |= (1 << 0); //Run led rood

	//EIMSK &= ~(1 << INT0); //disable DCC receive, ontvangst niet mogelijk. Nu met GPIOR0 bit 5
}
void stop() {

	TCCR2B &= ~(1 << 3); //disable interrupt prescaler
	TIMSK2 &= ~(1 << 1); //disable interrupt
	GPIOR0 &= ~(1 << 4); //motor vertragen. Vertragen/versnellen flag
	GPIOR0 &= ~(1 << 5); //Flag motor draait niet
	GPIOR1 &= ~(1 << 6); //enable switches
	PORTB &= ~(1 << 0); //Run led uit


	//EIMSK |= (1 << INT0); //enable DCC receive, ontvangst DCC mogelijk

	GPIOR2 |= (1 << 4); //draai15 flag, enable 180 turn
	draai15 = 0x00; // overbodig? dubbelop?
}
void noodstop() {
	//Noodstop, schakelt motor uit, haalt vergrendeling los
	GPIOR2 ^= (1 << 1); //Toggle noodstop aan of uit
	MOTOR(~GPIOR2 & (1 << 1));
}
void MOTOR(boolean offon) {
	//if (offon) {
	//	GPIOR0 |= (1 << 3); //on
	//}
	//else {
	//	GPIOR0 &= ~(1 << 3); //off
	//}
	//if (~GPIOR0 & (1 << 3)) { //Motor uit V5.02
	//Serial.println("M"); //bug25okt

	if (!offon) {
		GPIOR0 &= ~(1 << 3); //off
		PORTB |= (1 << 1); //disable motor, ENA van motor driver
		PORTD &= ~(1 << 4); //lock, vergrendeling los maken portdlaag
		stop();
		DSP_exe(20);
		GPIOR1 |= (1 << 6); //disable switches 

	}
	else {
		GPIOR0 |= (1 << 3); //on
		PORTB &= ~(1 << 1); //enable motor, ENA laag van de motordriver
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
			//Serial.println(melderadres); //25okt
			//if (melderadres > 0) GPIOR1 |= (1 << 4); //niet vrij geven als noodstop in melder bug25okt
		}
		RUN_home();
	}
	PRG_fase = 0;
}
void RUN_home() { //move to HOME
	//Serial.println("run");
	if (MEM_reg & (1 << 0) && melderadres > 0) { //Melder-mode	
		//Als brug in een stop staat niks doen in melder-mode
		stops_rq = melderadres - 1;
		DSP_exe(12);
		stop();
		lock();
		stops_current = stops_rq;
	}
	else { //home-mode of op tussengebied, geen active melder	
		runfase = 0;
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

	if (~GPIOR0 & (1 << 0)) { //GPIOR0 bit0 true homing, positie zoeken
		if (POS == 0 && ~MEM_reg & (1 << 0)) { //Alleen in home-mode stop motor 
			stop();
		}
		else if (POS == POS_rq) {

			if (MEM_reg & (1 << 0)) {//melder mode
				switch (runfase) {
				case 10:
					//Laatste deel gedraaid stop bereikt
					stop();

					//Onderstaande constructie is nog omdat de timerinterrupt erg snel gaat, en er minimaal 
					//code in de ISR moet worden uitgevoerd. Dit verplaatst de acties naar Loop()
					if (PRG_fase == 2) {
						//DSP_exe(16); //
						GPIOR1 |= (1 << 7); //set flag buiten ISR naar DSP_exe(16)
					}
					else {
						GPIOR2 |= (1 << 0); //zet flag, activeert Lock() in Loop						
					}
					break;
				}
			}
			else { //home mode
				stop();
				//Serial.print("�SR");
				PORTD |= (1 << 4); //lock bridge on	portdhoog			
			}
		}
		else if (POS == POS_calc) {
			if (MEM_reg & (1 << 0)) { //melder-mode
				if (runfase > 0)GPIOR1 |= (1 << 1); //call SLOW() via loop altijd in home-mode	
			}
			else {//home-mode
				GPIOR1 |= (1 << 1); //call SLOW() via loop altijd in home-mode	
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
	for (int i = 0; i < EEPROM.length(); i++) {
		EEPROM.update(i, 0xFF);
	}
}
void MEM_read() {
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
	if (aantalStops > 15)aantalStops = 4; //testen V3.0 moet zijn 8; bugs19nov aantalstops 15

	Vaccel = EEPROM.read(115);
	if (Vaccel > 10)Vaccel = 8;

	dr15ads = EEPROM.read(116); //aantal 'overkant' sporen in draai15 mode
	if (dr15ads > 8)dr15ads = 0;

	speling = EEPROM.read(117);
	if (speling > 254)speling = 10; //default speling, vrije slag in de aandrijving

	DCC_adres = EEPROM.read(103);
	if (DCC_adres == 0xFF)DCC_adres = 1;
	DCC_mode = EEPROM.read(104);
	if (DCC_mode > 10)DCC_mode = 1;

	//terug laden postities 
	for (byte i = 0; i < 16; i++) {
		EEPROM.get(200 + (5 * i), stops[i].pos); //200
		//EEPROM.get(300 + (5 + 1), stops[i].width);
		EEPROM.get(300 + (5 * i), stops[i].fine); //400
		//stops[i].reg = EEPROM.read(i + 1); //reg = register met 8 booleans
		//bit0 posities bepaald niet in EEPROM

		if (stops[i].pos == 0xFFFFFFFF) {
			stops[i].pos = (400000 * i) + 1000; //lift (peter)			
		}
	}
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
	byte stp;
	stp = stops_rq + 1;

	switch (txt) {
	case 10:
		regelst; display.print(F("Positie zoeken...."));
		break;

	case 12: //In bedrijf aanduiding
		regelb; display.print(stp);
		if (~GPIOR0 & (1 << 3)) { //motor uit
			regelst; display.print(F("X"));
		}
		if (melderadres > 0) {
			DSP_settxt(104, 5, 2);
			display.print(melderadres);
		}
		break;

	case 15://keuze in te stellen stops
		regel1s; display.print(F("stops instellen "));
		regel2;
		if (MEM_reg & (1 << 0) && melderadres == 0) { //Brug staat niet in een stop
			display.print(F("X"));
		}
		else {
			display.print(stp);
		}
		break;
	case 16: //instellen stops
		regel1s; display.print(F("Instellen stops ")); display.print(stp);
		regel2;		//display.print(POS);
		if (MEM_reg & (1 << 0)) { //melder-mode
			display.print(POS);
		}
		else { //home_mode
			display.print(POS);
		}
		break;

	case 20://motor off
		regelb; display.print("X");
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
				display.print(F("Single"));//iedere stop een adres
				break;
			case 1:
				display.print(F("Duo-RA")); //r=rechtdoor a=afslaand, twee stops per adres
				break;
			case 2:
				display.print(F("Duo-AR"));//a=afslaand r=rechtdoor
				break;
			case 3:
				display.print(F("Draai15"));
				break;
			}
			break;

		case 7:
			display.print(F("DCC "));
			regel2;
			display.print(F("Dr15>> "));
			display.print(dr15ads);
			break;
		case 8: //Speling vrije slag, slip,
			display.print(F(" speling")); regel2;
			display.print(speling);
			break;

		}
		break;

	case 50:
		regel1s; //display.print(F("Modes "));
		switch (PRG_level) {
		case 0:
			display.print(F("Posities met")); regel2;

			if (MEM_reg & (1 << 0)) {
				display.print(F("Melders"));
			}
			else {
				display.print(F("Home"));
			}
			break;
		case 1:
			display.print(F("Encoder richting")); regel2;

			if (MEM_reg & (1 << 1)) {
				display.print(F(">>"));
			}
			else {
				display.print(F("<<"));
			}
			break;
		case 2: //motor uit(stroomloos) bij bereiken stop
			display.print(F("Motor in stop")); regel2;
			if (MEM_reg & (1 << 2)) { //default
				display.print(F("aan"));
			}
			else {
				display.print(F("uit"));
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
		noodstop(); //aanpassing 16/7/2021

		//MOTOR(false);
		//DSP_exe(21); //12
		break;

	case 1://handmatig
		DSP_exe(30);
		break;
		/*
		Instellen stops
		2 opties:
			Melder-mode
			stop kiezen , stops_rq bevat keuze
			width stop opnieuw bepalen
			draai naar midden melder + fine
			toon display fine
			**instellen fine
			opslaan (knop3) of annuleren (knop4)

		Home-mode

		*/
	case 2: //instellen stops

		switch (PRG_level) {
		case 0: //kiezen stops met knoppen of encoder
			DSP_exe(15);
			break;

		case 1: //instellen gekozen stops

			if (MEM_reg & (1 << 0)) { //melder-mode
				if (melderadres - 1 == stops_rq) {

					//brug nu down draaien tot melder edge
					runfase = 40;
					up;
					OCR2A = Vhome;
					startmotor();
				}
			}
			else { //home-mode
				DSP_exe(16);
			}
			break;

		case 2: //opslaan fine instelling stops
			if (MEM_reg & (1 << 0)) { //melder-mode
				if (melderadres - 1 == stops_current) { //Alleen opslaan als brug nog BINNEN de melder-edges staat
					stops[stops_current].fine = POS;
					EEPROM.put(300 + (5 * stops_current), stops[stops_current].fine);
				}
			}
			else { //home-mode
				stops[stops_rq].pos = POS;
				EEPROM.put(200 + (5 * stops_rq), POS);
			}
			PRG_level = 0;
			DSP_exe(15);

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
			PORTD &= ~(1 << 7); //reset pin 7			
			break;
		case 1:
			DDRD &= ~(1 << 5);
			DDRD &= ~(1 << 7);
			DDRD |= (1 << 6);
			PORTD &= ~(1 << 6); //reset pin 6	
			break;
		case 2:
			DDRD &= ~(1 << 6);
			DDRD &= ~(1 << 7);
			DDRD |= (1 << 5);
			PORTD &= ~(1 << 5); //reset pin 5
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

		changed = sr ^ switchstatus[switchcount];
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
			//if (GPIOR1 & (1 << 2)) SW_melderadres(); //V5.02
			if (changed > 0 && switchcount == 1)SW_melderadres(); //melders veranderd
		}

	} //GPIOR1
}
void SW_melderadres() { //called from sw_read if MEM_reg bit 0 is true (mode-melders) and melderstatus changed
	melderadres = 0; bool meldervalid = false;
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

	//vanaf V5.02 melders moeten 2x goed worden gemeten voordat ze worden als valid worden gezien
	//nodig omdat niet altijd het adres precies gelijk op de melderpoorten komt. 

	if (oldmelder == melderadres) {	//mc26okt
		if (GPIOR2 & (1 << 5)) {
			meldervalid = true;
		}
		GPIOR2 &= ~(1 << 5); //reset flag count
	}
	else {
		oldmelder = melderadres;
		GPIOR2 |= (1 << 5);
		meldervalid = false;
	}

	if (!meldervalid) { //geen geldig melderadres
		switchstatus[1] = 0xFF; //genereerd direct nieuwe melder uitlezing
		if (~GPIOR1 & (1 << 2)) return;  //bug18nov20211
	}

	//if (GPIOR1 & (1 << 2))Serial.println("top");

	if (GPIOR1 & (1 << 2)) { //set in setup
		//Serial.println("jo");
		GPIOR1 &= ~(1 << 2);
		MOTOR(true);
	}

	if ((melderTemp > 0 && melderadres == 0) || (melderTemp == 0 && melderadres > 0)) {
		//dit dient om 'tussenmelderadres' onmogelijk te maken van 3 via 2 naar 0 bv...
		//hier VERANDERING van melder status
		MA_changed(); //Als bit3 =true melders uitschakelen, draaien naar POS_rq en stoppen. 
		DSP_exe(12);
		melderOld = melderadres;
	}


	melderTemp = melderadres;
}
void MA_changed() {
	byte stp;
	switch (runfase) {
	case 0: //zoeken naar stops
		if (melderadres == 0) { //melder vrijgekomen, breedte melder gemeten
			stp = melderOld - 1; //stop=melder-1
			if (stp != stops_current) {
				//call met speling correctie, 1x is van ricting veranderd
				F_width(stp, true); //Niet bij stop van vertrek, overstaande melder van deze stop moet gepasseerd zijn
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

	case 20:  //was 30 called from ISR2
		//Hier wordt het laatste stukje van edge naar stop berekend in normaal bedrijf, dus als stops bekend zijn.

		if (melderadres - 1 == stops_rq) {
			//gedraaid edge aankomst melder nu bereikt. 
			//stops_rq moet bekend zijn...
			if (GPIOR0 & (1 << 1)) {//updraaien POS_rq positief	
				POS_rq = ((stops[stops_rq].width / 2) + stops[stops_rq].fine);
				POS = 0;
			}
			else {//down draaien POS_rq negatief	
				POS = (stops[stops_rq].width / 2) - stops[stops_rq].fine;
				POS_rq = 0;
			}
			RUN_rq();
			runfase = 10;
		}

		break;



	case 40: //stops fine bepalen
		//in geval dat brug in stop staat eerst down draaien om melder edge te vinden
		if (melderadres == 0) { //melder komt vrij edge bereikt
			POS = 0;
			down;
			runfase = 41;
		}
		break;
	case 41:
		if (melderadres > 0) {
			runfase = 42;
		}
		break;
	case 42:
		stop();
		//call F-width zonder speling 2x is van richting veranderd wat de speling in de aandrijving weer (ongeveer) 0 maakt
		F_width(melderOld - 1, false);
		break;
	}
}
void F_width(byte stp, boolean splng) { //breedte van een melder bepaald
	stops[stp].reg &= ~(1 << 1); //sr26okt
	stops[stp].width = abs(POS); //altijd een positieve waarde
	if (stp == stops_rq) {
		GPIOR0 &= ~(1 << 5); //Flag motor draait niet
		//hier is van de stops_rq melder de width bekend	
		//Berekening POS_rq van edge naar stop tijdens het meten van de width. 
		//Berekening voor edge naar stop in normaal bedrijf gebeurt in MA_changed
		//Speling is de speling in de aandrijving, moet worden meegeteld als er van richting wordt veranderd,
		//Stepper maakt dan een x-aantal steps voordat de brug weer draait, de speling of vrije slag.	

		if (GPIOR0 & (1 << 1)) { //draait up
			POS_rq = calcPos(stp, true);
			if (splng)POS_rq = POS_rq - speling;
		}
		else {
			POS_rq = calcPos(stp, false);
			if (splng)POS_rq = POS_rq + speling;
		}
		RUN_rq();
		runfase = 10;
	}

	else {
		POS = 0;//reset position voor volgende meting
	}
}
long calcPos(byte stp, boolean updown) {
	POS = 0; //positie weer op nul stellen, nieuw pos request berekenen tov pos=0
	long result = 0;
	if (updown) {
		result = (stops[stp].width / 2) - stops[stp].fine;
		result = result * -1;
	}
	else {
		result = (stops[stp].width / 2) + stops[stp].fine;
	}
	return result;
}
void F_pos(byte stp) {
	stops[stp].reg &= ~(1 << 2); //sr26okt
	stops[stp].pos = abs(POS); // Altijd positief waarde
}
void SW_off(byte sw) {
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
			if (MEM_reg & (1 << 1)) { //richting encoder, meerdere types 
				SW_encoder(false); //V3.00 richting encoder instelbaar
			}
			else {
				SW_encoder(true); //V3.00 richting encoder instelbaar
			}
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
			if (MEM_reg & (1 << 1)) { //richting encoder, meerdere types 
				SW_encoder(true); //V3.00 richting encoder instelbaar
			}
			else {
				SW_encoder(false); //V3.00 richting encoder instelbaar
			}
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
		noodstop();
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
			//dient alleen tijdens bouwen om de motor te kunnen laten draaien.
			up;
			OCR2A = Vhome;
			startmotor();
			break;

		case 2: //instellen etages
			switch (PRG_level) {
			case 0:
				SW_encoder(true);
				break;
			case 1: //start motor
				if (MEM_reg & (1 << 0)) { //melder-mode
					OCR2A = Vmin;
					up;
					startmotor();
				}
				else { //home-mode
					OCR2A = Vhome;
					up;
					start();
				}
				break;
			}
			break;

		case 3: //Instellingen met byte waardes
			if (MEM_reg & (1<<7)) { //bug20nov
				MEM_reg ^= (1 << 6); //flip richting parameter waarde
				MEM_reg &=~(1 << 7); //reset flag knop1 ingedrukt
			}
			else {
				PRG_level++;
			}

			if (PRG_level > 8)PRG_level = 0;
			DSP_exe(40);
			break;

		case 4: //modes (MEM_reg instellingen)
			PRG_level++;
			if (PRG_level > 2)PRG_level = 0;
			DSP_exe(50);
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
			down;
			OCR2A = Vhome;
			startmotor();

			break;
		case 2: //instellen stops
			switch (PRG_level) {
			case 0:
				SW_encoder(false);
				break;

			case 1: //Motor down draaien om positie in te stellen
				if (MEM_reg & (1 << 0)) { //melder-mode
					down;
					OCR2A = Vmin;
					startmotor();
				}
				else { //Home_mode
					if (POS > 0) {
						OCR2A = Vhome;
						down;
						start();
					}
				}
				break;
			}
			break;

		case 3: //instellingen
			MEM_reg |= (1 << 7); //bug20nov, flag button parameter pressed
			SW_encoder(MEM_reg & (1 << 6)); // omhoog of omlaag
			//SW_encoder(false);

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
	//Altijd SW2 ON (ingedrukt)
	switch (PRG_fase) {
	case 0: //n bedrijf
		noodstop();
		//MOTOR(true, false); //toggled motor aan of uit. Ook vanuit setup...eenmalig 
		break;
	case 1:
		noodstop();
		//MOTOR(true, false);
		break;
	case 2: //instellen etages
		PRG_level++;
		DSP_prg();
		break;

	case 3: //instellen snelheden en diverse (waardes) instellingen 
		EEPROM.update(110, Vhome);
		EEPROM.update(111, Vmin);
		EEPROM.update(112, Vmax);
		EEPROM.update(113, Vstep);
		EEPROM.update(114, aantalStops);
		EEPROM.update(115, Vaccel);
		EEPROM.update(104, DCC_mode);
		EEPROM.update(116, dr15ads);
		EEPROM.update(117, speling);
		COM_V();
		OCR2A = Vhome;//????

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
	if (~GPIOR1 & (1 << 6)) { //disable switches in noodstop
	//if (~GPIOR0 & (1 << 5)) { //switch enabled, blocked als motor draait
	//tijdens positie testen ff uit
		PRG_fase++;
		PRG_level = 0;  //bug19nov
		if (PRG_fase > 5)PRG_fase = 0;
		DSP_prg();

		if (PRG_fase > 0) {
			free(); //Stel brug als bezet, groene led uit
		}
		else {
			MOTOR(true); //V3.00 herstart direct na aanpassingen
		}
	}
}
void ET_rq() {
	//PORTD &= ~(1 << 4); //free lock at new position request
	free(); //V3.00

	//if (~COM_reg & (1 << 0)) {
	//	COM_reg |= ((1 << 0));
	if (~GPIOR2 & (1 << 6)) {
		GPIOR2 |= ((1 << 6)); //m26okt


		runwait = millis(); //rw26okt
	}
	else {

		if (millis() - runwait > 6000) { //rw26okt
			if (MEM_reg & (1 << 0)) { //melders-mode
				if (stops_current != stops_rq)RUN_rq_M();
			}
			else { //home-mode
				POS_rq = stops[stops_rq].pos;
				RUN_rq();
			}
			//COM_reg &= ~(1 << 0);
			GPIOR2 &= ~(1 << 6); //m26okt
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
			//Serial.println("Run_rq");
			PORTD |= (1 << 4); //vrij geven portdhoog
		}
	} //	if (GPIOR0 & (1 << 5)) { //motor draait
}
void RUN_rq_M() { //called from et_rq (stops_rq==stopscurrent komt niet voor)
	boolean bekend = true;
	byte vertrek; byte aankomst;
	runfase = 0;
	//stops_rq=waar die naar toe moet, stop_current staat tie in
	POS = 0; //zet huidige positie op 0
	//Route berekening van middenpunt vertrekkende melder naar edge van aankomst melder.
	long dist = 0; //= stops[stops_current].width 2; //middelpunt vertrekkende melder meerekenen
	//Gebleken is dat het meenemen van de volledige breedte van de vertrekkende melder er mooier uitziet.
	//kijken of route bekend is
	if (stops_rq > stops_current) {
		up; // functioneel voor als bekend false blijkt te zijn, scheelt een if statement verderop (testen richting)
		aankomst = stops_rq;
		vertrek = stops_current;
	}
	else { //rq<current
		down; //zie up
		aankomst = stops_current;
		vertrek = stops_rq;
	}

	for (byte i = vertrek; i < aankomst; i++) { //alle route delen moeten bekend zijn.
		if (stops[i].reg & (1 << 2))bekend = false; //afstand tussen onderstop naar bovenstop (niet) bekend
		if (stops[i].reg & (1 << 1))bekend = false; //breedte melder (niet) bekend aankomst stop
		if (stops[i + 1].reg & (1 << 1))bekend = false; //breedte melder bekend vertrek stop
		dist = dist + stops[i].pos;
		dist = dist + stops[i].width; //breedte van de melder, ook vertrekkende melder breedte meegerekend
		//if (i != stops_rq) dist = dist + stops[i].width;  //breedte vertrekkende melder uitsluiten
	}
	/*
	Afstand is berekend van (middenpunt) vertrekkende melder tot edge aankomst melder. Daarna draait brug door in Vmin totdat
	de edge van de aankomst melder actief wordt. Pos wordt dan weer 0 gezet (in F_width) en de afstand van de edge tot het juiste stoppunt
	wordt berekend en daarna naar toe gedraait.
	Deze eerste draai is een ongeveer draai. Blokkade of slippen van de brug heeft hierdoor geen invloed op stopplek, draaien
	gaat door totdat de aankomst melder actief wordt.
	*/

	if (bekend) {
		runfase = 20; //zie MA_changed()
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
		OCR2A = Vhome;
		startmotor();
	}
}
void SW_encoder(boolean dir) {
	switch (PRG_fase) {
	case 0:
		if (~GPIOR1 & (1 << 6)) { //als motor draait keuze nieuwe stop niet mogelijk
			ENC_select(dir);
			ET_rq();
			DSP_exe(12);
		}
		break;
	case 1: //handmatig
		ENC_fine(dir);
		DSP_exe(30);
		break;

	case 2: //stops instellen V3.00
		switch (PRG_level) {
		case 0: //in te stellen stop kiezen, niet voor melder-mode
			if (~MEM_reg & (1 << 0)) {
				ENC_select(dir);
			}
			DSP_exe(15);
			break;
		case 1: //Instellen stop
			//Instelling van de ENCODER knoppen doen nu iets anders
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
				if (aantalStops < 1) aantalStops = 15;
				break;
			case 6: //DCC modi
				DCC_mode--;
				if (DCC_mode > 3)DCC_mode = 3;
				break;
			case 7: //Draai15 mode aantal 'overkant' sporen
				if (dr15ads > 0)dr15ads--;
				break;
			case 8:
				if (speling > 0)speling--;
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
				if (aantalStops > 15)aantalStops = 1;
				break;
			case 6: //DCC modi
				DCC_mode++;
				if (DCC_mode > 3)DCC_mode = 0;
				break;
			case 7: //in draai15 mode aantal 'overkant' sporen
				dr15ads++;
				if (dr15ads > 15)dr15ads = 0;
				break;
			case 8: //speling, vrije slag
				if (speling < 254)speling++;
				break;
			}
		}
		DSP_exe(40);
		break;
	case 4: //mode instellen MEM_reg
		MEM_reg ^= (1 << PRG_level);
		//level1  true=melder-mode false=home-mode
		//level2  Encoder richting
		//level3 motor mode
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
	//Serial.println("L"); //25okt
	stops_current = melderadres - 1;
	//if (~GPIOR1 & (1 << 4)) { //bug25okt
	PORTD |= (1 << 4); //Lock pin hoog 
	//Serial.println("G"); //25okt
//}
//GPIOR1 &= ~(1 << 4); //reset noodstop flag
	runfase = 0;

	if (~MEM_reg & (1 << 2)) { //motor uit in stop
		GPIOR2 |= (1 << 2); //zet timer aan, timer zet motor uit na 2 seconde in Loop()
		runwait = millis(); //rw26okt
	}
}
void free() {
	//Maakt de brug vrij om te kunnen draaien
	GPIOR0 |= (1 << 3);// flag voor motor enabled V5.02
	PORTB &= ~(1 << 1);// enable motor
	PORTD &= ~(1 << 4); //groene led, lock low  portdlaag

}
void ENC_fine(boolean dir) {
	//Actie van de ENCODER, knop functies zijn anders

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
	Dcc.process();

	GPIOR2 ^= (1 << 7); //flip counter
	if (GPIOR2 & (1 << 7)) slowcount++;//Counter voor langzame processen 1xin 512 cycli
	//timing hier is tricky, werking switches en encoder 

	if (slowcount == 0) { //V5.02 op 500, balans tussen bouncing switches of encoder goed werken
		//slowcount = 0;

		if ((GPIOR0 & (1 << 5)) && (GPIOR0 & (1 << 4)))FAST(); //versnellen
		if (GPIOR1 & (1 << 1))SLOW(); //vertragen
		count++;
		if (count == 0) {
			if (GPIOR0 & (1 << 6))RUN_rq();
			//if (COM_reg & (1 << 0))ET_rq();
			if (GPIOR2 & (1 << 6))ET_rq(); //26okt
		}
		//Acties uit ISR2 TIMER 
		if (GPIOR1 & (1 << 7)) {
			GPIOR1 &= ~(1 << 7); //reset flag
			//POS = 0; //nieuwe begin positie om fine te kunnen meten
			POS = stops[stops_current].fine;
			DSP_exe(16); //instellen fine stops, toont stops[].fine
		}

		if (GPIOR2 & (1 << 0)) {
			GPIOR2 &= ~(1 << 0); //reset flag
			lock(); //
		}
		//timer  voor uitschakelen motor in lock na periode  OPM.runwait wordt ook in et_rq() gebruikt
		if (GPIOR2 & (1 << 2)) {
			if (millis() - runwait > 3000) { //timer 2 seconden rw26okt
				GPIOR2 &= ~(1 << 2);
				GPIOR0 &= ~(1 << 3); //Motor enabled flag clear V5.02
				PORTB |= (1 << 1); //disable motor
				DSP_exe(12);
			}
		}
		//Volgorde in LOOP is hier belangrijk, SW_read onderaan....
		SW_read();
	}
}
