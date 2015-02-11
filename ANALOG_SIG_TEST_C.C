/*==================================================================*
 *Written by: 	Prokopov L. LDI
 *Date:			17.03.2006	Ver. 1.00
 *File:			analog_sig_test.c
 *For :			RMC2300
 *Project:     Narciss
 *Status:      developing
 *             16 bit ACD with SPI interface with flash lamp
 *===================================================================*/
//#use "user.lib"

#define I2CSCLBit 4			//Port D
#define I2CSDABit 5
#define i2cRetries 100

#use "I2C_DEVICES.LIB"

//#define DAC_ADDRESS 0x2c
//#define DAC_CHN		0
#define DAC_COMMAND 0x09

#define CINBUFSIZE  31
#define COUTBUFSIZE 31
#define BAUDRATE	57600

/* Output bits defenition		*/
// Port A
#define AD_CS		0				//
#define EN_LAMP		1
#define DAC_CS		3
//#define SIP			5				//L.P. for 64009 -> on OUT port
#define D_DAC		7				// Input data bit for DAC

// Port B
#define D_MAIN		2				//input, data bit for main channel
#define D_REF		5
#define AD_CLK		7

//OUT
#define OUT			0x8000 	// for PE4
#define SIP			4
			//
#define DAC_WD				16			//bit; command + data
#define DAC_RES			8			//bit
#define TEST_GAIN			255	//150		//65
#define DAC_MIDL_VALUE	128
#define ADC_RES			16          //bit
#define PERIOD				9			//ms, 19ms = 47.17Hz, 9ms = 95.54Hz
#define ACCM				32
#define ACCM_PW			5
#define MAX_POINTS		100		//200

char serviceByte;
char dataAD[ADC_RES];
char OUTShadow;

void initPorts();
void usDelay ( int iDelay );
void MsDelay(int iDelay);
void readADC (void);
unsigned int getData(char chann);
void SetDAC(char command, char value);
//=========================================================
void main(){
	char i, value, points;
	unsigned int AD_ref, AD_main;
	unsigned long data_Sum_ref, data_Sum_main;
//   unsigned long time;

	initPorts();

   value = 60;
	SetDAC(DAC_COMMAND, value);
   BitWrPortI(PADR, &PADRShadow, 1, EN_LAMP);

   points = MAX_POINTS;
//   	while (points--){
	while (1){
   		data_Sum_ref = data_Sum_main = 0;
        //collect data for accumulation number
	   	for (i = 0; i < ACCM; i++){
				BitWrPortE(OUT, &OUTShadow, 1, SIP);
				usDelay(10);
				readADC ();
				BitWrPortE(OUT, &OUTShadow, 0, SIP);	//lamp sinhro impuls
      		MsDelay (PERIOD);

 //        printf("Data from ADC = %x.\n",AD_data);
		   	data_Sum_ref += getData(D_REF);
         	data_Sum_main += getData(D_MAIN);
			}
        //Find and display average
      printf("Sum from ADC main = %ul.\n",data_Sum_main);
      printf("Sum from ADC ref = %ul.\n",data_Sum_ref);
		AD_ref = (unsigned int)(data_Sum_ref >> ACCM_PW);
        if (!bit(&serviceByte, D_REF)){
      		printf("Data from ADC ref = %u.\n", AD_ref);
//      		printf("%u\n", AD_ref);
        }
        else{
        	printf("Channel REF don't connect.\n");
			;
        }
      	AD_main = (unsigned int)(data_Sum_main >> ACCM_PW);
        if (!bit(&serviceByte, D_MAIN)){
      		printf("Data from ADC main = %u.\n\n", AD_main);
//      		printf("%u\n", AD_main);
        }
        else{
	        	printf("Channel MAIN don't connect.\n\n");
			;
        }
/*
      sprintf(temp,"Data ref = %d \n", AD_ref);
      serCputs (temp);
      sprintf (temp, "main = %d \n", AD_main );
      serCputs (temp);
*/
		MsDelay(100);
   }
}
//===========================================================
void initPorts()
{
/************* Port A init **********************************/
//	WrPortI(SPCR, &SPCRShadow, 0x80);		//port A input
	WrPortI(SPCR, &SPCRShadow, 0x84); 		// port A as an output port
	WrPortI(PADR, &PADRShadow, 0xff);		// all high

	BitWrPortI(PADR, &PADRShadow, 0, SIP);
	BitWrPortI(PADR, &PADRShadow, 0, EN_LAMP);

	BitWrPortI(PBDR, &PBDRShadow, 0, AD_CLK);
/************* Port C init **********************************/
	WrPortI(PCFR, &PCFRShadow, 0xFF);		//all port C as serial communications
/************* Prot E init **********************************/
	WrPortI(PEFR, &PEFRShadow, 0x90);		// bit 4,7 is an I/O strobe
	WrPortI(IB4CR, &IB4CRShadow, 0xa8);		// I/O strobe:  address 0x8000,
// allow writes, 3 wait state, WR strob
	WrPortI(IB7CR, &IB7CRShadow, 0x10);		// I/O strobe:  address 0xe000,
//notallow writes, 1 wait state, RD strob
	WrPortI(PECR, &PECRShadow, 0x00);		// no clocked outputs
	WrPortI(PEDR, &PEDRShadow, 0xcc);		// all outputs are high
	WrPortI(PEDDR, &PEDDRShadow, 0xFC);		// bits 0-1 are inputs

	serCopen(BAUDRATE);								//RS232 C open
}

void usDelay ( int iDelay )
{
	int i;
	iDelay /= 11;
	for ( i=0; i<iDelay; i++ );
}

void MsDelay(int iDelay)
{
	unsigned long currTime, startTime, delay;
   char wait;

   wait = 1;
   startTime = MS_TIMER;
   delay = startTime + iDelay;
   while( wait ){
      currTime = MS_TIMER;
      if( currTime >= startTime ){
      	if( currTime > delay )
         	wait = 0;
      }
      else{
      	if( currTime > iDelay )
         	wait = 0;
      }
   }
}
//ADS8321
#define SERV_BIT 6
void readADC (){
   char i;

	BitWrPortI(PADR, &PADRShadow, 0, AD_CS);		//select ADS

   for (i = 0; i < (ADC_RES + SERV_BIT); i++){
      BitWrPortI(PBDR, &PBDRShadow, 1, AD_CLK);
      if (i > (SERV_BIT - 1)){
      	dataAD[i - SERV_BIT] = RdPortI(PBDR);
//         printf("Port B[%d] %x.\n", (i- 6), dataAD[i - 6]);
      }
      else{
      	serviceByte = RdPortI(PBDR);
      }
//      usDelay_a(5);
      BitWrPortI(PBDR, &PBDRShadow, 0, AD_CLK);
   }

   BitWrPortI(PADR, &PADRShadow, 1, AD_CS);
}

unsigned int getData(char chann){
	auto char bit_numb, bit_mask;
   auto unsigned int data;

   data = 0;
   bit_mask = 0;
   bit_mask = (1 << chann);
   for (bit_numb = 0; bit_numb < ADC_RES; bit_numb++){
//		printf("Temp %d %x.\n", i, temp[i]);
		//port B bit x channel
     	if (dataAD[bit_numb] & bit_mask){
			data |= 0x0001;
        }
     	else{
			data &= 0xfffe;
        }

     	if (bit_numb < (ADC_RES - 1)){
			data <<= 1;
        }
//	      printf("data %x.\n",data);
   }
//	printf("ADC data %x.\n",data);
   //convert format from two complement
   data += 32767;
   data += 1;
//      printf("data %d.\n",data);

	return data;
}
//DAC MAX550 OK 20.12.2014
void SetDAC(char command, char value){
	auto char i, compl_value;

	compl_value = 0xFF - value;
	BitWrPortI(PADR, &PADRShadow, 0, DAC_CS);
	for(i = 0; i < DAC_WD; i++){
    	if (i < (DAC_RES)){
        	if (command & 0x80)
				BitWrPortI(PADR, &PADRShadow, 1, D_DAC);
        	else
				BitWrPortI(PADR, &PADRShadow, 0, D_DAC);

    		command <<= 1;
		}
		else {
        	if (compl_value & 0x80)
				BitWrPortI(PADR, &PADRShadow, 1, D_DAC);
			else
				BitWrPortI(PADR, &PADRShadow, 0, D_DAC);

			compl_value <<= 1;
		}
		BitWrPortI(PBDR, &PBDRShadow, 1, AD_CLK);
		usDelay(5);
		BitWrPortI(PBDR, &PBDRShadow, 0, AD_CLK);
	}
	BitWrPortI(PADR, &PADRShadow, 1, DAC_CS);

	return;
}