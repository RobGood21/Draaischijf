/*
 Name:		Draaischijf.ino
 Created:	12/29/2018 8:26:39 PM
 Author:	Rob Antonisse

 Opmerkingen:
 werking S88 via opto TL072 rail op 5 S88 poort op 6, richting DCC is (soms) belangrijk, niet duidelijk waarom.
 Servo tijdens reset, starten arduino is er overspraak, oplossen door een 1K tussen puls en +5V
 Huidige brug geeft fysiek positie 6 als positie 7 hiervoor correctie in void SW_cl en in void setup




*/

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

//**declaration for SERVO
byte SERVO_reg = 0;

unsigned long SERVO_stoptime;
unsigned long SERVO_time;
unsigned int SERVO_pos;
unsigned long SERVO_clock;
unsigned int SERVO_min = 1500;//unlocked
unsigned int SERVO_max = 900;//locked

//DCC adres in this version NOT programmable because project is only for private model railroad layout, can be set by setting value of DCCadres
//For used adresses check void  APP_DCC()
unsigned long SW_time;
//unsigned long SW_periode;
byte COM_reg;
byte MOT_mode;
unsigned long MOT_time;
byte POS_last; //last posion on place
byte POS_cur; // current position
byte POS_rq; //requested position
byte POS_reset = 100; //postion auto rewind
byte resettimer;
byte S88_ct;
byte fase;
byte portc;

void setup() {

	//**begin Setup for DeKoder 
	Serial.begin(9600); //only needed if APP_monitor is used
	//interrupt on PIN2
	DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//External Interrupt Mask Register bit0 INT0 > 1
	//**End Setup for DeKoder

	//**for servo
	DDRD |= (1 << 7); //set PIN7 as output
	PORTD |= (1 << 7); //set servo port high
	DDRD |= (1 << 6); //set PIN6 as output for S88 control


	DDRB = 0xFF; //port B as output
	DDRC = 0x00; //port C as input
	PORTC |= (1 << 0); PORTC |= (1 << 1); PORTC |= (1 << 2); //pins A0 -A2 set pullup resister


	//initialise start values for bridge
	portc = PINC;
	POS_cur = portc >> 3;
	if (POS_cur == 7)POS_cur = 6;
	POS_last = POS_cur;
	COM_reg |= (1 << 4); //bridge inactive wait for command

	POS_reset = 100 + POS_cur - 3; // 3=zero track

	Serial.print("Startpositie: ");
	Serial.print(POS_cur);
	Serial.print("   Reset: ");
	Serial.println(POS_reset);
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

					POS_rq = POS_last - 1;
					if (POS_rq < 1)POS_rq = 6;
					ST_start(1);
					break;
				case 1:
					//schijf stop
					//ST_stop(); cannot be used because of closing servo
					//manual stop opens servo emergency stop
					MOT_mode = 0; //stops stepper
					ST_clear(); //sets coils of stepper off
					SERVO_open();
					S88_lock(true);

					//reset rewind count, makes current position start point
					POS_reset = 100;
					COM_reg |= (1 << 4); //stop bridge

					break;
				case 2:
					//schijf rechtom (zwart)
					POS_rq = POS_last + 1;
					if (POS_rq > 6)POS_rq = 1;
					ST_start(2);
					break;
				}
			}
		}
		changed = changed >> 3;
		if (changed > 0) {
			POS_cur = PINC >> 3;
			if (POS_cur == 7)POS_cur = 6;
			if (POS_cur > 0 & POS_last != POS_cur) {
				//verdraaiing hier meten
				if (bitRead(COM_reg, 1) == true) {
					POS_reset--;
				}
				else {
					POS_reset++;
				}
				Serial.print("Pos_reset:  ");
				Serial.println(POS_reset);
				POS_last = POS_cur;
			}
			ST_position();
		}
	}
	portc = PINC;
}
void ST_stop() {
	MOT_mode = 0;
	ST_clear();
	SERVO_close();
}
void ST_start(byte direction) {
	//true==right, false=left

	Serial.print("Position:  ");
	Serial.println(POS_cur);


	Serial.print("request: ");
	Serial.println(POS_rq);
	//Serial.println(direction);
	COM_reg &= ~(1 << 0); //lock request, reset only possible
	COM_reg &= ~(1 << 3); //lock S88

	COM_reg &= ~(1 << 4); //bridge active

	S88_lock(true);
	SERVO_open();
	switch (direction) {
	case 0: //stop
		ST_stop();
		break;
	case 1: //turn right
		MOT_mode = 1;
		COM_reg |= (1 << 1); //set direction memory
		break;
	case 2: //turn left
		MOT_mode = 2;
		COM_reg &= ~(1 << 1); //clear direction memory
		break;
	}

}
void ST_position() {
	//Serial.println(POS_cur);
	if (MOT_mode > 0) {
		if (POS_cur > 0) {

			if (POS_cur == POS_rq) { //reached correct position
				ST_stop();
				COM_reg &= ~(1 << 2); //reset correction bit
				COM_reg |= (1 << 3); //set timer for S88 clear
				S88_ct = 0;
			}
			else {
				if (bitRead(COM_reg, 2) == true) {
					//in this case only possible situation is that the bridge turns wrong direction to make correction, so change direction.
					COM_reg ^= (1 << 1); //toggle direction memory
					if (bitRead(COM_reg, 1) == true) {
						ST_start(1);
					}
					else {
						ST_start(2);
					}
				}
			}
		}
	}
	else {
		/*
		positie verandert zonder dat de schrijf draait, hier een foutafhandeling maken.
		stel servo mist de walllock dan krijgen we hier een nieuwe ongewenste positie
		eerst de servo terugtrekken,
		Correctie bit com_reg bit 2 zetten
		daarna draairichting omkeren en herstarten

*/
		if (bitRead(COM_reg, 4) == false) {
			Serial.println("positie veranderd zonder beweging");
			SERVO_open();
			COM_reg |= (1 << 2); //set correction bit
			COM_reg ^= (1 << 1); //toggle direction memory
			COM_reg &= ~(1 << 3);//reset S88 timer clear

			if (bitRead(COM_reg, 1) == true) {
				ST_start(1);
			}
			else {
				ST_start(2);
			}
		}

	}
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
void APP_Monitor(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//application for DCC monitor
	if (type == true) {
		Serial.print("CV:   , ");
	}
	else {
		Serial.print("Switch, ");
	}
	Serial.print("Adres: ");
	Serial.print(adres);
	Serial.print("(");
	Serial.print(decoder);
	Serial.print("-");
	Serial.print(channel);
	Serial.print("), ");
	//cv
	if (type == true) {
		Serial.print("CV: ");
		Serial.print(cv);
		Serial.print(", waarde: ");
		Serial.print(value);
	}
	else {
		if (port == true) {
			Serial.print("A<, ");
		}
		else {
			Serial.print("R>, ");
		}
		if (onoff == true) {
			Serial.print("On.");
		}
		else {
			Serial.print("Off");
		}
	}
	Serial.println("");
}

void COM_exe(boolean type, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//type=CV(true) or switch(false)
	//decoder basic adres of decoder 
	//channel assigned one of the 4 channels of the decoder (1-4)
	//Port which port R or L
	//onoff bit3 port on or port off
	//cv cvnumber
	//cv value
	int adres;
	adres = ((decoder - 1) * 4) + channel;
	//Applications 
	//APP_Monitor(type, adres, decoder, channel, port, onoff, cv, value);
	APP_DCC(type, adres, decoder, channel, port, onoff, cv, value);
	//Add a void like APP_monitor for application
}

//**End void's for DeKoder
void APP_DCC(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	/*
	DCC command station sends many commands and rehearsels of commands, much effort is needed to clean this up. To get a solid sequence of commands.
	*/
	static byte check;
	byte received;
	if (decoder > 27 & decoder < 30) {
		//received = type + adres + channel + port;
		//if ((check ^ received) > 0) {
			//check = received;
			switch (adres) {
			case 109: //als brug vrij is is automatisch terugdraaien mogelijk
				if (port == false) {
					Serial.println("brugspoor is vrij");
					COM_reg &= ~(1 << 5);
				}
				else {
					Serial.println("brugspoor is bezet");
					COM_reg |= (1 << 5);
				}
				break;

			case 110:
				if (port == true) {
					Serial.println("reset");
					POS_rq = 0;
					COM_reg |= (1 << 0);
				}
				break;

			case 111:
				if (port == true) {
					Serial.println("bit1");
					if (bitRead(COM_reg, 0) == true) POS_rq |= (1 << 0);
				}
				break;
			case 112:
				if (port == true) { //bit2
					Serial.println("bit2");
					if (bitRead(COM_reg, 0) == true)POS_rq |= (1 << 1);
				}
				break;
			case 113:
				if (port == true) { //bit4
					Serial.println("bit4");
					if (bitRead(COM_reg, 0) == true)POS_rq |= (1 << 2);
					break;
			case 114:
				if (port == true & bitRead(COM_reg, 0) == true) {
					//COM_reg &= ~(1 << 0);
					//Serial.println(POS_rq);
					ST_start(1);
				}
				break;
				}
			case 115:
				if (port == true & bitRead(COM_reg, 0) == true) {
					//COM_reg &= ~(1 << 0);
					//Serial.println(POS_rq);
					ST_start(2);
					break;
				}
			}
		//}
	}
}



void S88_lock(boolean lock) {
	//set pin 6 for S88 control
	if (lock == true) {
		PORTD |= (1 << 6);
	}
	else {
		PORTD &= ~(1 << 6);
	}

}
void SERVO_close() {
	SERVO_pos = SERVO_max;
	SERVO_reg |= (1 << 2);
	SERVO_stoptime = millis();
}

void SERVO_open() {
	SERVO_pos = SERVO_min;
	SERVO_reg |= (1 << 2);
	SERVO_stoptime = millis();
}
void ST_autoback() { //called from loop
	if (bitRead(COM_reg, 5) == false & bitRead(COM_reg, 4) == true) {
		if (POS_reset < 95) ST_start(2); //is halve draai plus 1 verder
		if (POS_reset > 105)ST_start(1);
	}
}
void SERVO_run() {
	//SERVO_speed();
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
	//**place in loop for DeKoder
	DEK_DCCh();
	//**end Loop DeKoder
	if (bitRead(SERVO_reg, 2) == true) {
		SERVO_run();
		if (millis() - SERVO_stoptime > 1000) {
			SERVO_reg &= ~(1 << 2);
			PORTD |= (1 << 7); //set servo port high
		}
	}

	if (millis() - MOT_time > 20) {
		MOT_time = millis();
		if (MOT_mode > 0 & bitRead(SERVO_reg, 2) == false) ST_step();
	}

	if (millis() - SW_time > 20) {
		SW_time = millis();
		SW_cl();


		resettimer++;
		if (resettimer == 0) { //once in 3 seconds
			ST_autoback();
		}




		if (bitRead(COM_reg, 3) == true) {
			S88_ct++;
			if (S88_ct > 100) {
				S88_lock(false); //give bridge free
				COM_reg |= (1 << 4); //bridge stop, inactive
				COM_reg &= ~(1 << 3);
			}
		}
	}
}
