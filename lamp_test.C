/*================================================================*
 *Written by: 	Prokopov L. LDI
 *Date:			08.01.2005	Ver. 1.00
 *File:			Lamp_test.c
 *For :			RMC2300
 *Project:     ISP64
 *Status:      developing
 *				Lamp test
 *================================================================*/
#define OUT			0x8000 	// for PE4
#define DUMM		0xa000	// PE5 for read encoders
#define IN			0xe000	// PE7

/* Output bits defenition		*/
// Port A
#define AD_CS		0				//
#define EN_L		1				//enable lamp

/* Other			*/
// OUT
#define SIP			4	//L.P. was 5				//lamp sinchoimpuls

//14.12.2014
//Inputs
#define INDX		0				//index from stepper motor encoders
#define SW_HOME	2				//home switch for motors
#define BLOCK		4				//block HV for PMT
#define SW_END		5				//end switch for motors
#define CELL		7				//cell insirt switch

#define DAC_COMMAND 0x09
#define DAC_CS		3
#define D_DAC		7			//Port A bit 7 - data
#define AD_CLK		7			//Port B bit 7
#define GAIN		60

char OUTShadow;

void SetDAC(char command, char value);
void usDelay(int iDelay );
void MsDelay(int iDelay);
// port init function
void initPorts ();
//////////////////////////////////////////////////////////////////////////////
void main(){
//06.01.2014
	int switch_state;
//
	initPorts();

//    SetDAC(DAC_COMMAND, GAIN);
    BitWrPortI(PADR, &PADRShadow, 1, EN_L);

	for ( ; ;){

		BitWrPortE(OUT, &OUTShadow, 1, SIP);	//lamp sinhro impuls
   		usDelay(300);
		BitWrPortE(OUT, &OUTShadow, 0, SIP);	//lamp sinhro impuls
		MsDelay(10);
/*/
//14.12.2014
		switch_state = BitRdPortE(IN, SW_HOME + 1);
      if (switch_state)
      	printf("Mot1 isn't in home pos.\n");
      else
      	printf("Mot1 is in home pos.\n");
*/
   }
/*
//test for little-big endian
   uint16_t test;
   test = 1;
	printf("%s\n", *((unsigned char *) &test) == 0 ? "big-endian" : "little-endian");
  	return 0;
*/
}
//==========================================================================//
nodebug
void initPorts(void)
{
/************* Port A init **********************************/
//	WrPortI(SPCR, &SPCRShadow, 0x80);		//port A input
	WrPortI(SPCR, &SPCRShadow, 0x84); 		// port A as an output port
	WrPortI(PADR, &PADRShadow, 0x5d);		// all high, SIP, EN_L, D_DAC low
/************* Port B init **********************************/
/************* Port C init **********************************/
	WrPortI(PCFR, &PCFRShadow, 0xFF);		//all port C as serial communications
/************* Port D init ******************** see I2C.LIB */
/************* Prot E init **********************************/
	WrPortI(PEFR, &PEFRShadow, 0xb0);		// bit 4,7 is an I/O strobe
	WrPortI(IB4CR, &IB4CRShadow, 0xa8);		// I/O strobe:  address 0x8000,
// allow writes, 3 wait state, WR strob
	WrPortI(IB7CR, &IB7CRShadow, 0x10);		// I/O strobe:  address 0xe000,
//notallow writes, 1 wait state, RD strob
	WrPortI(PECR, &PECRShadow, 0x00);		// no clocked outputs
	WrPortI(PEDR, &PEDRShadow, 0xcc);		// all outputs are high
	WrPortI(PEDDR, &PEDDRShadow, 0xFC);		// bits 0-1 are inputs

//	serCopen(57600);								//RS232 C open
	return;
}

void SetDAC(char command, char value){
	auto char a, i;

   a = 0;
   BitWrPortI(PADR, &PADRShadow, 0, DAC_CS);
	for(i = 0;i < 16;i++){
      if (i < 8){
         if (command & 0x80)
				BitWrPortI(PADR, &PADRShadow, 1, D_DAC);
         else
				BitWrPortI(PADR, &PADRShadow, 0, D_DAC);

         command <<= 1;
      }
      else {
         if (value & 0x80)
				BitWrPortI(PADR, &PADRShadow, 1, D_DAC);
         else
				BitWrPortI(PADR, &PADRShadow, 0, D_DAC);

         value <<= 1;
      }
      BitWrPortI(PBDR, &PBDRShadow, 1, AD_CLK);
      usDelay(5);
      BitWrPortI(PBDR, &PBDRShadow, 0, AD_CLK);
   }
   BitWrPortI(PADR, &PADRShadow, 1, DAC_CS);

	return;
}

nodebug
void usDelay ( int iDelay )
{
	int i;

    if(iDelay > 11){
		iDelay /= 11;
    }
    else{
		iDelay = 1;
    }

	for ( i = 0; i < iDelay; i++ ){
    	;
    }
}

void MsDelay(int iDelay)
{
	unsigned long currTime, startTime, delay;
	char wait;

   	wait = TRUE;
	startTime = MS_TIMER;
	delay = startTime + iDelay;
	while( wait ){
    	currTime = MS_TIMER;
    	if( currTime >= startTime ){
      		if( currTime > delay ){
            	wait = FALSE;
            }
		}
		else{
      		if( currTime > iDelay ){
         		wait = FALSE;
            }
		}
	}
}