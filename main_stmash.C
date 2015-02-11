/*==========================================================*
 *Written by: 	Prokopov L.
 *Date:			20.11.2014	Ver. 1.00
 *File:			nt5_new_main.c
 *For :			RMC2300
 *Project:     nt5_new
 *Status:		developing
 *				New firmware to nt5
 *==========================================================*/
#use boardtypes.lib
#use "user.lib"
#use "TMC222.lib"


#define PRINTABLE		1

#define CINBUFSIZE  15
#define COUTBUFSIZE 15

// This timeout period determines when an active input data stream is considered
// to have ended and is implemented within serBread. Will discontinue collecting
// data 3 seconds after receiving any character or when maxSize are read.
#define MSSG_TMOUT 3000UL

// This timeout period determines when to give up waiting for any data to
// arrive and must be implemented within the user's program, as it is below.
// Will timeout after 5 seconds if no data is read.
#define IDLE_TMOUT 5000UL

#define BAUDRATE	115200

/* Output bits defenition		*/
// Port A
#define AD_CS			0
#define EN_LAMP		1
#define DAC_CS			3
// Input data bit for DAC
#define D_DAC			7

// Port B
//input, data bit for main channel
#define D_MAIN		2
#define D_REF		5
#define AD_CLK		7

// Port D
#define I2CSCLBit 4
#define I2CSDABit 5
#define i2cRetries 100
#use I2C.LIB

#define	MAXSIZE	255
//External port address definition
// for PE4
#define OUT			0x8000
// PE5 for read encoders
#define DUMM		0xa000
// PE7
#define IN			0xe000
// OUT
//encoders byte select; 0 - MSB
#define SEL				0
//reset encoders
#define EM_ENC_NRES	1
#define EX_ENC_NRES	2
//lamp sinchro impuls
#define SIP				4

// IN
//index from stepper motor encoders
#define EM_ENC_INDX		0
#define EX_ENC_INDX		1
//home switch for motors
#define EM_SW_HOME		2
#define EX_SW_HOME		3
//HV block switch
#define BLOCK				4
//end switch for motors
#define EM_SW_END			5
#define EX_SW_END			6
//cell insirt switch
#define CELL				7

//set PMT HV
#define GAIN		60

#define DAC_COMMAND 0x09

// Motots control
#define TMC_ADDR	0xC0
//the last bit in addres fild is write/read (RD = 1)
#define RD_BIT		0x01

// All commands for TMC222
#define	GETFULLSTATUS1		0x81	//return complete status of the chip
#define	GETFULLSTATUS2		0xfc	//return actual, target and secure position
#define	GETOTPPARAM			0x82	//return OPT parameters
#define	GOTOSECUREPOS		0x84	//drive motor to secure position
#define	HARDSTOP				0x85	//immediate full stop
#define	RESETPOS				0x86	//set actual position to zero
#define	RESETTODEFAULT		0x87	//overwrite the chip RAM with OPT contents
#define	RUNINIT				0x88	//reference search
#define	SETMOTORPARAM		0x89	//set motor parameters
#define	SETOTPPARAM 		0X90	//zap the OTP memory
#define	SETPOSITION			0x8b	//programm the target and secure position
#define	SOFTSTOP				0x8f	//motor stop with deceleration phase

// Motor parameters for Narciss
#define	VMAX			0x0e	//index 14 grupp D 726 FS/s (11658 1/16 microsteps)
#define	VMIN        0x0c	//index 12 271 FS/s
#define	ACC			0x08	//was a,index 10 acceleration 21886 FS/s2
#define	IRUN			0x08	//8 238mA	for new P21h4aa motors 09.11.2005
#define	IHOLD			0x00	//59mA

typedef	unsigned char	byte_t;
typedef	unsigned int	uint16_t;
typedef	unsigned short	ushort32_t;
typedef	unsigned long	ulong64_t;

//typedef struct {
const	byte_t head[3] = {'m', 's', 'g'};
//}msgHead_t;

typedef struct {
// y-coordinate, number of logical excitation steps (zero-based)
	byte_t ex_step;
// x-coordinate, number of logical emission steps (zero-based)
	byte_t em_step;
}pointCoord_t;

typedef struct {
// accumulation for each point
	uint16_t accumulation;
// set of bits, now in use only flagBkgrSubtr(0x0001)
	uint16_t flags;
// number of measurement points
	uint16_t nmb_of_points;
// points_ExEm[nmb_of_points] => {points_ExEm[0], points_ExEm[1], ...
// points_ExEm[nmb_of_points-1]}
	pointCoord_t *points_ExEm;
}measParams_t;

typedef struct {
	measParams_t params;					//
	float *values;						// values[params.nmb_of_points]
}measData_t;

byte_t mssg_buff[MAXSIZE];

typedef struct {
   byte_t* cmdHead;
   uint16_t cmd_size;
//command code
   byte_t cmd;
   measParams_t params;
   byte_t cs;
}cmdMsg_t;

cmdMsg_t message;

typedef struct {
	byte_t* rspHead;
   uint16_t rsp_size;
//status code
   byte_t st;
   measData_t data;
   byte_t cs;
}rspMsg_t;

rspMsg_t respond;
//16 microsteps
const char mot_param[] = {((IRUN << 4)|IHOLD), ((VMAX << 4)|VMIN),
									ACC, 0x00, 0x0c};
//-32768 microsteps
const char home_pos[] = {0x80, 0x00};
//32767 microsteps
const char end_pos[]  = {0x7f, 0xff};
//-251 microsteps ??? //TODO define
const char beg_pos[]  = {0xff, 0x05};
//State mashine
enum MStates {IDLE, RECIVE, TRANSMIT, DECODE, EXECUTE};

enum Mot {FIRST, SECOND = 2};
enum Command {GET_STATUS = 1, SET_PARAMS, START_MEASURE, GET_DATA, STOP_MEASURE};
enum Status {OK = 1, BUSY, ERROR, DATA_READY, DATA};
//Global variables
byte_t OUTShadow;
// port init function
void clear(byte_t* buff, byte_t buff_size);
void SetDAC(byte_t command, byte_t value);
/* Functions				*/
void Dev_Init();
void Reset_m(byte_t mot);

/******************************************************************************/
main() {
	enum MStates mState;
	enum Command cmd;
   enum Status status;

   byte_t i, rx_bytes;
   byte_t accm_num;
   byte_t stp_size;
   byte_t num_of_stp;

   uint16_t tx_bytes;
   ulong64_t t;

   respond.rspHead = head;
   respond.rsp_size = 0;
   respond.st = status;

   message.cmd_size = 0;
   clear(mssg_buff, MAXSIZE);

	Dev_Init();
#ifdef	PRINTABLE
	printf("Device init.\n");
#endif

	for (; ;)
	{
	   switch (mState)
	   {
	     case IDLE:
         mState = RECIVE;
	     break;
	     case RECIVE:
#ifdef	PRINTABLE
			printf("Recieve state.\n");
#endif
			t = MS_TIMER;
			rx_bytes = serCread(mssg_buff, MAXSIZE, MSSG_TMOUT);
//Check message size and check summ
			if (rx_bytes > 0){
         	printf("%s.\n", mssg_buff);
            mState = DECODE;
         }
         else
         	mState = IDLE;
	     break;
	     case TRANSMIT:
#ifdef	PRINTABLE
			printf("Transmit state.\n");
#endif
         if (rx_bytes > 0){
         	respond.rsp_size = 1;
         	respond.cs = ~(respond.rsp_size ^ respond.st);
            tx_bytes = sizeof(respond.rspHead) + respond.rsp_size + 1;
  		   	serCwrite(&respond, tx_bytes);                         //(byte_t*)
         }
  		   tx_bytes = 0;
         mState = IDLE;
	     break;
	     case DECODE:
        	mState = EXECUTE;
	     break;
	     case EXECUTE:
        	mState = TRANSMIT;
	     break;
	     default:
        	mState = IDLE;
	   }
	}			//endless loop
}				//end main
/******************************************************************************/
nodebug
void clear(char* buff, byte_t buff_size){
	byte_t i;

   for (i = buff_size; i != 0; --i)
   {
   	*buff++ = 0;
   }
}

/********* Device init *****************************************/
//nodebug
void Dev_Init()
{
	byte_t i;

	initPorts();
   //All output pins - low
	WrPortE(OUT,&OUTShadow,0x00);

   SetDAC(DAC_COMMAND, GAIN);
	//Warmup lamp
   BitWrPortI(PADR, &PADRShadow, 1, EN_LAMP);

	for (i = 0; i < 100; ++i){
		BitWrPortE(OUT, &OUTShadow, 1, SIP);	//lamp sinhro impuls
   	usDelay(330);
		BitWrPortE(OUT, &OUTShadow, 0, SIP);	//lamp sinhro impuls
		MsDelay(10);
   }
#ifdef	PRINTABLE
	printf("Reset motors start.\n");
#endif
	Reset_m(FIRST);
	MsDelay(100);
	Reset_m(SECOND);

   return;
}

void SetDAC(byte_t command, byte_t value){
	byte_t a, i;

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
      usDelay(11);
      BitWrPortI(PBDR, &PBDRShadow, 0, AD_CLK);
   }
   BitWrPortI(PADR, &PADRShadow, 1, DAC_CS);

	return;
}

//nodebug
void Reset_m(byte_t mot)
{
	byte_t input;

   byte_t motorAddr;

   motorAddr = mot;
	readMotorStatus(GETFULLSTATUS1, motorAddr);
	send_TMC_Command((TMC_ADDR | motorAddr), RESETPOS);
   readMotorStatus(GETFULLSTATUS2, motorAddr);
//write motor parameters
   send_TMC_Param((TMC_ADDR | motorAddr), SETMOTORPARAM, &mot_param[0],
   						sizeof(mot_param));
   readMotorStatus(GETFULLSTATUS1, motorAddr);

//motor to home position
#ifdef	PRINTABLE
	printf("Mot %d go to home position \n", motorAddr);
#endif
	send_TMC_Param((TMC_ADDR | motorAddr), SETPOSITION, &home_pos[0],
   						sizeof(home_pos));
   if (FIRST == mot){
	   while (BitRdPortE(IN, EM_SW_HOME)){
	           ;
	   }
   }
   else {
	   while (BitRdPortE(IN, EX_SW_HOME)){
	           ;
	   }
   }
   send_TMC_Command((TMC_ADDR | motorAddr), HARDSTOP);

   input = RdPortE(IN);
#ifdef	PRINTABLE
   printf("Motor %d in home_pos (input = %x)\n", motorAddr, input);
#endif

	readMotorStatus(GETFULLSTATUS1, motorAddr);
	readMotorStatus(GETFULLSTATUS2, motorAddr);
	send_TMC_Command((TMC_ADDR | motorAddr), RESETPOS);
/*
//Reset encoder
	    BitWrPortE(OUT, &OUTShadow, 0, EM_ENC_NRES);
	    BitWrPortE(OUT, &OUTShadow, 1, EM_ENC_NRES);
*/
   readMotorStatus(GETFULLSTATUS2, motorAddr);

	return;
}