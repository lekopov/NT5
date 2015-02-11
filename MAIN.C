/*==========================================================*
 *Written by: 	Prokopov L.
 *Date:			20.11.2014	Ver. 1.00
 *File:			nt5_new_main.c
 *For :			RMC2300
 *Project:     nt5_new
 *Status:		developing
 *				New firmware to nt5
 *				protocol v.1c , little endean
 *==========================================================*/
#memmap	xmem

#use boardtypes.lib
#use "user.lib"
#use "TMC222.lib"

#define PRINTABLE			1
//#define PRINTABLE_MO		1
//#define PRODUCT			0

#define CINBUFSIZE  1023
#define COUTBUFSIZE 255

// This timeout period determines when an active input data stream is considered
// to have ended and is implemented within serXread. Will discontinue collecting
// data 0.5 seconds after receiving any character or when maxSize are read.
#define MSSG_TMOUT 500UL

// This timeout period determines when to give up waiting for any data to
// arrive and must be implemented within the user's program, as it is below.
// Will timeout after 5 seconds if no data is read.
#define IDLE_TMOUT 5000UL

// Define time towarm up lamp 1 min
#define WARM_TIME 60000

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

#define MAXSIZE				1024
#define MAX_RX_SIZE			1024
#define MAXPOINTS				16384
#define MAX_DATA_POINTS		256
#define MAX_MOT_POSITION	30000
//head size + status + body size(alwas zero)
#define MIN_RESPOND_SIZE	6
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
//#define EM_ENC_NRES	1
//#define EX_ENC_NRES	2
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
//#define EM_SW_END			5
//#define EX_SW_END			6
//cell insirt switch
#define CELL				7

//set PMT HV
#define GAIN		60

#define DAC_COMMAND 0x09

//ADS8321
#define SERV_BIT 			6
#define ADC_RES			16          //bit

#define PERIOD				9			//ms, 19ms = 47.17Hz, 9ms = 95.54Hz

#define BODY_OFFSET			6
#define ST_OFFSET				3
#define CMD_OFFSET			3
#define SERVICE_BYTES		6
//global flags defenetion
#define FLG_NO_PARMS			0x01
#define FLG_NO_DATA			0x02
//#define FLG_DATA_RDY			0x04
// Motots control
#define TMC_ADDR	0xC0
//the last bit in addres fild is write/read (RD = 1)
#define RD_BIT		0x01
//Accumulation definetios
#define ACCM_8		8
#define ACCM_32	32
#define ACCM_64	64
#define ACCM_128	128
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
/* 07.02.2015 L.P. define new params to motor  * /
#define	VMAX			0x0e	//index 14 grupp D 726 FS/s (11658 1/16 microsteps)
#define	VMIN        0x0c	//index 12 271 FS/s
#define	ACC			0x08	//was a,index 10 acceleration 21886 FS/s2
*/
#define	VMAX			0x0c  //456
#define	VMIN        0x0f  //214
#define	ACC			0x0f	//40047
#define	IRUN			0x08	//8 238mA	for new P21h4aa motors 09.11.2005
#define	IHOLD			0x00	//59mA

typedef	unsigned char	byte_t;
typedef	unsigned int	uint16_t;
typedef	unsigned long	ulong32_t;

enum Mot {FIRST, SECOND = 2};
enum Command {GET_STATUS = 1, SET_PARAMS, START_MEASURE, GET_DATA, STOP_MEASURE};
enum Status {OK = 1, BUSY, ERROR, DATA_READY, DATA};
enum ErrorCode {CS_ERR = 1, CMD_ERR, PRM_ERR, CTX_ERR};

const	byte_t head[3] = {'m', 's', 'g'};

typedef struct {
// gain, set PMT HV
	byte_t gain;
// accumulation for each point
	byte_t accumulation;
// set of bits, now in use only flagBkgrSubtr(0x0001)
	byte_t flags;
// number of logical steps for the full diapason of ex/em gratings, allowed
// values: 1..255. Defines the resolution for Ex/Em
	byte_t ex_log_steps;
   byte_t em_log_steps;
// number of measurement regionss
// allowed values: 1..(not defined, reject/correct dynamically by number of regionss)
	byte_t nmb_of_regions;
   struct{
// Ex/Em start point for start measurement, in logical steps
	   byte_t ex_start_p;
	   byte_t em_start_p;
// number of excitation/emission steps
	   byte_t ex_steps;
	   byte_t em_steps;
   }points[CHAR_MAX];
}measParams_t;

measParams_t params;

typedef struct {
	measParams_t *meas_param;					//
	uint16_t *values;						// values[params.nmb_of_regions]
}measData_t;

typedef struct {
	   byte_t num_of_ex_stp;
     	byte_t num_of_em_stp;
     	byte_t ex_stp_size;
	   byte_t em_stp_size;
}pointsPrm_t;

byte_t rx_mssg[MAX_RX_SIZE];

union {
	byte_t respond[(MAXSIZE << 1) + MIN_RESPOND_SIZE];
   struct
   {
   	byte_t head[3];
      byte_t stt;
      uint16_t msgsize;
      byte_t ercode;
   }rsp;
}tx_mssg;

byte_t dataAD[ADC_RES];

//16 microsteps
const char mot_param[] = {((IRUN << 4)|IHOLD), ((VMAX << 4)|VMIN),
									ACC, 0x00, 0x0c};
//-32768 microsteps
const char home_pos[] = {0x80, 0x00};
//32767 microsteps
const char end_pos[]  = {0x7f, 0xff};
//300 microsteps for ex, 2600 for em
const char ex_beg_pos[]  = {0x01, 0x2c};
const char em_beg_pos[]  = {0x0a, 0x28};
#define EX_BEG_POSITION	300
//116
#define EX_MIN_STEP_SIZE (MAX_MOT_POSITION - EX_BEG_POSITION)/(MAX_DATA_POINTS - 1)

#define EM_BEG_POSITION	2600
//107
#define EM_MIN_STEP_SIZE (MAX_MOT_POSITION - EM_BEG_POSITION)/(MAX_DATA_POINTS - 1)

//Global variables
byte_t OUTShadow;
byte_t executed;
byte_t gFlags;

uint16_t num_of_steps_exec, num_of_steps;
uint16_t ex_initial_pos, em_initial_pos;
uint16_t ex_target_pos, em_target_pos;
uint16_t ref_data[MAX_DATA_POINTS], main_data[MAXSIZE];
// port init function
void clear(byte_t* buff, uint16_t buff_size);
void SetDAC(byte_t command, byte_t value);
void readADC ();
unsigned int getData(char chann);
char isPositionSet(unsigned int MotorAddr, unsigned int position);
/* Functions				*/
byte_t calcCS(byte_t* mssg, uint16_t number_of_bytes);
uint16_t swapBytes(uint16_t src);

cofunc int Dev_Init();
cofunc int Reset_m[2](byte_t mot);
cofunc int Data_meas(byte_t accumulation, pointsPrm_t* prm);

CoData Recieve, Transmit, Decode, Exec;
/******************************************************************************/
main() {
//	enum Command cmd;
//   enum Status status;

   byte_t i, k, clientCmd;
   byte_t csumm;
//	byte_t gain, accm_num;
   byte_t* temp;

   int result;
   uint16_t n, prm_length, val;
   uint16_t tx_bytes, rx_bytes;
   uint16_t summ_of_ex_stp, summ_of_em_stp;

   ulong32_t var_temp;

   //exitation and emission steps size in step motor's microsteps
   pointsPrm_t rg_measPar;

   clear((byte_t*)&tx_mssg, sizeof(tx_mssg));
   strncpy(tx_mssg.rsp.head, head, sizeof(head));
//   tx_mssg.msgsize = 1;
//   tx_mssg.respond[ST_OFFSET] = (byte_t)OK;

   clear(rx_mssg, MAX_RX_SIZE);
   n = (MAXSIZE << 1);
   clear((byte_t*)&main_data, n);
	clear((byte_t*)&ref_data, (MAX_DATA_POINTS << 1));

	for (; ;)
	{
		costate Dev_Int init_on{
#ifdef	PRINTABLE
			printf("Device init.\n");
#endif
			CoPause(&Recieve);
         CoPause(&Transmit);
			CoPause(&Decode);
			CoPause(&Exec);
			wfd Dev_Init();
         CoResume(&Recieve);
		}
		costate Recieve always_on{
#ifdef	PRINTABLE
			printf("Recieve state.\n");
#endif
			wfd rx_bytes = cof_serCread(rx_mssg, MAX_RX_SIZE, MSSG_TMOUT);

			if (rx_bytes > 0){
#ifdef	PRINTABLE
         	printf("%s .\n", rx_mssg);
#endif
//Check message head
				result = strncmp(rx_mssg, head, sizeof(head));
            if (result != 0){
            	tx_mssg.rsp.stt = (byte_t)ERROR;
               tx_mssg.rsp.msgsize = 1;
               tx_mssg.rsp.ercode = (byte_t)CTX_ERR;

               CoBegin(&Transmit);
            }
            else {
#ifdef	PRINTABLE
					for (i = 0; i < (rx_bytes - sizeof(head)); ++i){
         			printf("%x ", rx_mssg[sizeof(head) + i]);
            	}
               printf("\n");
#endif
            	CoBegin(&Decode);
				}
			}
   	}
		costate Decode{
#ifdef	PRINTABLE
			printf("Decode state.\n");
#endif
			clientCmd = rx_mssg[CMD_OFFSET];

         if (isCoRunning(&Exec) && ((clientCmd != (byte_t)GET_STATUS) &&
         							(clientCmd != (byte_t)STOP_MEASURE))){
            tx_mssg.rsp.stt = (byte_t)ERROR;
            tx_mssg.rsp.msgsize = 1;
            tx_mssg.rsp.ercode = CTX_ERR;

            CoReset(&Decode);
            CoBegin(&Transmit);
            yield;
          }

         switch (clientCmd)
         {
           case (byte_t)GET_STATUS:
#ifdef	PRINTABLE
				printf("Command GET_STATUS.\n");
#endif
            if (isCoRunning(&Exec)){
#ifdef	PRINTABLE
            	printf("Exec is running.\n");
#endif
//Calculate persentage for Execution
               var_temp = (ulong32_t)num_of_steps_exec * 100;
					executed = (byte_t)(var_temp / num_of_steps);

					tx_mssg.rsp.stt = (byte_t)BUSY;
               tx_mssg.rsp.msgsize = 1;
               tx_mssg.respond[SERVICE_BYTES] = executed;

               CoBegin(&Transmit);
            }
            else {
					tx_mssg.rsp.stt = (byte_t)OK;
				}
           break;
           case (byte_t)SET_PARAMS:
#ifdef	PRINTABLE
				printf("Command SET_PARAMS.\n");
#endif
//Check cs first, start from params body
				prm_length = rx_bytes - SERVICE_BYTES - 1;
            csumm = calcCS(&rx_mssg[BODY_OFFSET], prm_length);
            if ((csumm ^ rx_mssg[rx_bytes - 1]) == 0){
	            memcpy((byte_t*)&params, &rx_mssg[BODY_OFFSET], prm_length);
//               		(sizeof(params) - sizeof(params.points)));
//					memcpy((byte_t*)&params.points, &rx_mssg[BODY_OFFSET + (sizeof(params)
//               		- sizeof(params.points))], sizeof(params.points));

               if (params.accumulation != ACCM_8 && params.accumulation != ACCM_32 &&
               	params.accumulation != ACCM_64 && params.accumulation != ACCM_128){
						tx_mssg.rsp.stt = (byte_t)ERROR;
                  tx_mssg.rsp.msgsize = 1;
                  tx_mssg.rsp.ercode = PRM_ERR;

               	CoBegin(&Transmit);
                  CoReset(&Decode);
               }
               else {
#ifdef   PRINTABLE
	            	printf(" params.accumulation = %d.\n", params.accumulation);
#endif
//               	accm_num = params.accumulation;
						;
               }

               if (params.ex_log_steps == (MAX_DATA_POINTS - 1))
               {
               	rg_measPar.ex_stp_size = EX_MIN_STEP_SIZE;
               }
               else if (params.ex_log_steps != 0 &&
               			params.ex_log_steps < MAX_DATA_POINTS){
               	rg_measPar.ex_stp_size = MAX_MOT_POSITION / params.ex_log_steps;
               }
               else if (params.ex_log_steps == 0){
               	rg_measPar.ex_stp_size = 0;
               }
               else {
#ifdef   PRINTABLE
	            	printf(" params.ex_log_steps = 0.\n");
#endif
						tx_mssg.rsp.stt = (byte_t)ERROR;
						tx_mssg.rsp.msgsize = 1;
                  tx_mssg.rsp.ercode = (byte_t)PRM_ERR;

               	CoBegin(&Transmit);
                  CoReset(&Decode);
					}

               if (params.em_log_steps == (MAX_DATA_POINTS - 1))
               {
               	rg_measPar.em_stp_size = EM_MIN_STEP_SIZE;
               }
               else if (params.em_log_steps != 0 &&
               			params.em_log_steps < MAX_DATA_POINTS){
               	rg_measPar.em_stp_size = MAX_MOT_POSITION / params.em_log_steps;
               }
               else if (params.em_log_steps == 0){
               	rg_measPar.em_stp_size = 0;
               }
               else {
#ifdef   PRINTABLE
	            	printf(" params.em_log_steps = 0.\n");
#endif
						tx_mssg.rsp.stt = (byte_t)ERROR;
                  tx_mssg.rsp.msgsize = 1;
                  tx_mssg.rsp.ercode = (byte_t)PRM_ERR;

               	CoBegin(&Transmit);
                  CoReset(&Decode);
					}
               summ_of_ex_stp = summ_of_em_stp = 0;
               for (k = 0; k < params.nmb_of_regions; ++k){
               	summ_of_ex_stp += params.points[k].ex_steps;
               	summ_of_em_stp += params.points[k].em_steps;
               }
/*L.P. if add 1 we have more than 1024 TBD */
               num_of_steps = summ_of_ex_stp * summ_of_em_stp;
//Should be less than 1024;
					if (num_of_steps > MAXPOINTS){
#ifdef   PRINTABLE
	            	printf("Number of steps a = %d (%4x) more than MAXSIZE.\n",
                  		num_of_steps, num_of_steps);
#endif
						tx_mssg.rsp.stt = (byte_t)ERROR;
                  tx_mssg.rsp.msgsize = 1;
                  tx_mssg.rsp.ercode = (byte_t)PRM_ERR;

               	CoBegin(&Transmit);
                  CoReset(&Decode);
               }
//*/
#ifdef   PRINTABLE
	            printf("Flags f = %d (%4x).\n", params.flags, params.flags);
	            printf("Number of Ex points n = %d (%4x).\n", params.ex_log_steps,
               			params.ex_log_steps);
               printf("Number of Em points n = %d (%4x).\n", params.em_log_steps,
               			params.em_log_steps);
					printf("Step size for Ex n = %d (%4x).\n", rg_measPar.ex_stp_size,
               			rg_measPar.ex_stp_size);
               printf("Step size for Em n = %d (%4x).\n", rg_measPar.em_stp_size,
               			rg_measPar.em_stp_size);
#endif
//	            tx_mssg.rsp.msgsize = 1;
	            tx_mssg.rsp.stt = (byte_t)OK;
               gFlags &=~(FLG_NO_PARMS);
               gFlags |= FLG_NO_DATA;
            }
            else {
					tx_mssg.rsp.stt = (byte_t)ERROR;
               tx_mssg.rsp.msgsize = 1;
               tx_mssg.rsp.ercode = (byte_t)CS_ERR;

//               CoBegin(&Transmit);
            }
           break;
           case (byte_t)START_MEASURE:
#ifdef	PRINTABLE
				printf("Command START_MEASURE.\n");
#endif
				if ((gFlags & FLG_NO_PARMS) != FLG_NO_PARMS){
	            tx_mssg.rsp.stt = (byte_t)OK;

	            CoBegin(&Exec);
            }
            else {
					tx_mssg.rsp.stt = (byte_t)ERROR;
               tx_mssg.rsp.msgsize = 1;
               tx_mssg.rsp.ercode = (byte_t)PRM_ERR;

//               CoBegin(&Transmit);
            }
           break;
        	  case (byte_t)GET_DATA:
#ifdef	PRINTABLE
				printf("Command GET_DATA.\n");
#endif
	         if ((gFlags & FLG_NO_DATA) != FLG_NO_DATA){
	            temp = &tx_mssg.respond[BODY_OFFSET];
//Send meas parameters first
					memcpy(temp, (byte_t*)&params, prm_length);
               temp += prm_length;

	            if (rg_measPar.num_of_ex_stp ==0 && rg_measPar.num_of_em_stp == 0){
	               *temp = (byte_t)(main_data[0] >> 8);
	               ++temp;
	               *temp = (byte_t)main_data[0];
	               ++temp;
	               *temp = (byte_t)(ref_data[0] >> 8);
	               ++temp;
	               *temp = (byte_t)ref_data[0];
	               ++temp;
	               val = 4;
	            }
	            else {
	               for (i = 0; i < rg_measPar.num_of_ex_stp; ++i){
	                  for (k = 0; k < rg_measPar.num_of_em_stp; ++k){
	                     *temp = (byte_t)(main_data[i * rg_measPar.num_of_em_stp + k] >> 8);
	                     ++temp;
	                     *temp = (byte_t)main_data[i * rg_measPar.num_of_em_stp + k];
	                     ++temp;
	                  }
	                  *temp = (byte_t)(ref_data[i] >> 8);
	                  ++temp;
	                  *temp = (byte_t)ref_data[i];
	                  ++temp;
	               }
	               val = (uint16_t)(rg_measPar.num_of_ex_stp + 1) * (uint16_t)(rg_measPar.num_of_em_stp);
                  val <<= 1;
	            }
	            tx_mssg.rsp.msgsize = prm_length + val;
	            tx_mssg.rsp.stt = (byte_t)DATA;
	            *temp = calcCS(&tx_mssg.respond[BODY_OFFSET], tx_mssg.rsp.msgsize);
//Add one byte to CS
               ++tx_mssg.rsp.msgsize;
	         }
	         else {
					tx_mssg.rsp.stt = (byte_t)ERROR;
               tx_mssg.rsp.msgsize = 1;
               tx_mssg.rsp.ercode = (byte_t)CTX_ERR;
	         }
//            CoBegin(&Transmit);
           break;
           case (byte_t)STOP_MEASURE:
#ifdef	PRINTABLE
				printf("Command STOP_MEASURE.\n");
#endif
				CoReset(&Exec);
	         ex_initial_pos = ex_target_pos  = (uint16_t)(ex_beg_pos[0] << 8);
	         ex_initial_pos = ex_target_pos |= (uint16_t)(ex_beg_pos[1]);

	         em_initial_pos = em_target_pos  = (uint16_t)(em_beg_pos[0] << 8);
	         em_initial_pos = em_target_pos |= (uint16_t)(em_beg_pos[1]);

	         goToPos(FIRST, ex_target_pos);
	         goToPos(SECOND, em_target_pos);
	         waitfor (isPositionSet(FIRST, ex_target_pos));
	         waitfor (isPositionSet(SECOND, em_target_pos));

				tx_mssg.respond[ST_OFFSET] = (byte_t)OK;
//            CoBegin(&Transmit);
           break;
           default:
#ifdef	PRINTABLE
				printf("Command doesn't found!\n");
#endif
					tx_mssg.rsp.stt = (byte_t)ERROR;
               tx_mssg.rsp.msgsize = 1;
               tx_mssg.rsp.ercode = (byte_t)CMD_ERR;
         }
			CoPause(&Recieve);
         CoBegin(&Transmit);
	  	}				//end	decode

  		costate Transmit{
#ifdef	PRINTABLE
			printf("Transmit state.\n");
#endif
         rx_bytes = 0;
         clear(rx_mssg, MAX_RX_SIZE);

         if ((GET_DATA == clientCmd) &&
         		(FLG_NO_DATA != (gFlags & FLG_NO_DATA)))
            tx_bytes = MIN_RESPOND_SIZE + val + 1;
         else if (tx_mssg.rsp.msgsize == 1){
//Send error code to client
            tx_bytes = MIN_RESPOND_SIZE + 1;
         }
         else
            tx_bytes = MIN_RESPOND_SIZE;

#ifdef   PRINTABLE_MO
         for (n = 0; n < tx_bytes; ++n){
            printf("0x%2x.\n",tx_mssg.respond[n]);
         }
#endif
         wfd cof_serCwrite(tx_mssg.respond, tx_bytes);

  		   tx_bytes = 0;
         clear((byte_t*)&tx_mssg.rsp.stt, (sizeof(tx_mssg.rsp) - sizeof(tx_mssg.rsp.head)));
			CoResume(&Recieve);
	   } 	//end transmit costate

		costate Exec{		// execute
   //		CoPause(&Recieve);

#ifdef	PRINTABLE
			printf("Start execution\n");
#endif
//			tx_mssg.respond[ST_OFFSET] = (byte_t)BUSY;

//  		   CoBegin(&Transmit);
// Main execution programm
         for (k = 0; k < params.nmb_of_regions; ++k){
	         ex_target_pos += (params.points[k].ex_start_p * rg_measPar.ex_stp_size);
	         em_target_pos += (params.points[k].em_start_p * rg_measPar.em_stp_size);

	         goToPos(FIRST, ex_target_pos);
	         goToPos(SECOND, em_target_pos);

            rg_measPar.num_of_ex_stp = params.points[k].ex_steps;
            rg_measPar.num_of_em_stp = params.points[k].em_steps;

	         waitfor (isPositionSet(FIRST, ex_target_pos));
	         waitfor (isPositionSet(SECOND, em_target_pos));

	         SetDAC(DAC_COMMAND, params.gain);
	         wfd Data_meas(params.accumulation, &rg_measPar);

	         ex_target_pos = ex_initial_pos;
	         em_target_pos = em_initial_pos;
         }

         CoResume(&Exec);
         gFlags &= ~(FLG_NO_DATA);
			tx_mssg.rsp.stt = (byte_t)DATA_READY;

  		   CoBegin(&Transmit);
   	}		//end exec costate
	}			//endless loop
}				//end main
/******************************************************************************/
nodebug
void clear(char* buff, uint16_t buff_size){
	uint16_t i;

   for (i = buff_size; i != 0; --i)
   {
   	*buff++ = 0;
   }
}

/********* Device init *****************************************/
//nodebug
cofunc int Dev_Init()
{
	byte_t i;
	ulong32_t t1, t2;

	initPorts();
   //All output pins - low
	WrPortE(OUT,&OUTShadow,0x00);
   gFlags = FLG_NO_PARMS + FLG_NO_DATA;
#ifdef	PRINTABLE
	printf("Reset motors start.\n");
#endif
	wfd Reset_m[0](FIRST);
//	waitfor(DelayMs(100));
	wfd Reset_m[1](SECOND);

   ex_initial_pos = em_initial_pos = 0;
   ex_target_pos = em_target_pos = 0;

  	ex_initial_pos = ex_target_pos  = (uint16_t)(ex_beg_pos[0] << 8);
  	ex_initial_pos = ex_target_pos |= (uint16_t)(ex_beg_pos[1]);

  	em_initial_pos = em_target_pos  = (uint16_t)(em_beg_pos[0] << 8);
  	em_initial_pos = em_target_pos |= (uint16_t)(em_beg_pos[1]);

   goToPos(FIRST, ex_target_pos);
   goToPos(SECOND, em_target_pos);
   waitfor (isPositionSet(FIRST, ex_target_pos));
   waitfor (isPositionSet(SECOND, em_target_pos));

   SetDAC(DAC_COMMAND, GAIN);
	//Warmup lamp
   BitWrPortI(PADR, &PADRShadow, 1, EN_LAMP);

#ifdef PRODUCT
	t1 = t2 = MS_TIMER;
   t1 += WARM_TIME;
   while (t2 < t1){
	   BitWrPortE(OUT, &OUTShadow, 1, SIP);   //lamp sinhro impuls
	   usDelay(330);
	   BitWrPortE(OUT, &OUTShadow, 0, SIP);   //lamp sinhro impuls
      waitfor(DelayMs(10));
    	t2 = MS_TIMER;
   }
#else
//Replace this by time
	for (i = 0; i < 250; ++i){
		BitWrPortE(OUT, &OUTShadow, 1, SIP);	//lamp sinhro impuls
   	usDelay(330);
		BitWrPortE(OUT, &OUTShadow, 0, SIP);	//lamp sinhro impuls
		waitfor(DelayMs(10));
   }
#endif
	BitWrPortI(PADR, &PADRShadow, 1, EN_LAMP);

	return 1;
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
cofunc int Reset_m[2](byte_t mot)
{
	byte_t input;
   byte_t motorAddr;
   byte_t status[9];

   motorAddr = mot;
	readMotorStatus(GETFULLSTATUS1, motorAddr, status);
	send_TMC_Command((TMC_ADDR | motorAddr), RESETPOS);
   readMotorStatus(GETFULLSTATUS2, motorAddr, status);
//write motor parameters
   send_TMC_Param((TMC_ADDR | motorAddr), SETMOTORPARAM, &mot_param[0],
   						sizeof(mot_param));
   readMotorStatus(GETFULLSTATUS1, motorAddr, status);

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

	readMotorStatus(GETFULLSTATUS1, motorAddr, status);
	readMotorStatus(GETFULLSTATUS2, motorAddr, status);
	send_TMC_Command((TMC_ADDR | motorAddr), RESETPOS);

	return 1;
}

byte_t calcCS(byte_t* mssg, uint16_t number_of_bytes)
{
	byte_t i, xor_summ;

   xor_summ = 0;
   for (i = number_of_bytes; i > 0; --i){
		xor_summ ^= *mssg;
      ++mssg;
   }
	return ~xor_summ;
}

uint16_t swapBytes(uint16_t src)
{

	uint16_t result;

   result = 0;
   result = ((src << 8) & 0xff00) | ((src >> 8) & 0x00ff);

	return result;
}

cofunc int Data_meas(byte_t accumulation, pointsPrm_t* prm)
{
	byte n, i, k;

   uint16_t initial_pos;
   uint16_t ref_temp[MAX_DATA_POINTS];

   ulong32_t ref, main, background;

#ifdef	PRINTABLE
   printf("Data measurement start.\n");
#endif

	num_of_steps_exec = 0;
   initial_pos = em_target_pos;

   if (prm->num_of_ex_stp ==0 && prm->num_of_em_stp == 0){
//Measurement for one point
		n = accumulation;

		do {
			if (params.flags){
				readADC();
				background += getData(D_MAIN);
			}
//Flash lamp sinhro impuls
			BitWrPortE(OUT, &OUTShadow, 1, SIP);
			usDelay(330);
			readADC();
			BitWrPortE(OUT, &OUTShadow, 0, SIP);

			ref += getData(D_REF);
			main += getData(D_MAIN);

			waitfor(DelayMs(PERIOD));
		} while (--n);

		if (params.flags){
			main -= background;
		}

      switch (accumulation)
      {
        case ACCM_8:
        		ref_data[0] = (uint16_t)(ref >> 3);
            main_data[0] = (uint16_t)(main >> 3);
        break;
        case ACCM_32:
        		ref_data[0] = (uint16_t)(ref >> 5);
            main_data[0] = (uint16_t)(main >> 5);
        break;
        case ACCM_64:
        		ref_data[0] = (uint16_t)(ref >> 6);
            main_data[0] = (uint16_t)(main >> 6);
        break;
        case ACCM_128:
        		ref_data[0] = (uint16_t)(ref >> 7);
            main_data[0] = (uint16_t)(main >> 7);
        default: ;
      }
	}
	else{
		for (i = 0; i < prm->num_of_ex_stp; ++i){
			BitWrPortI(PADR, &PADRShadow, 1, EN_LAMP);
			for (k = 0; k <  prm->num_of_em_stp; ++k){
				n = accumulation;

				do {
					if (params.flags){
						readADC();
						background += getData(D_MAIN);
					}
	//Flash lamp sinhro impuls
					BitWrPortE(OUT, &OUTShadow, 1, SIP);
					usDelay(330);
					readADC();
					BitWrPortE(OUT, &OUTShadow, 0, SIP);

					ref += getData(D_REF);
					main += getData(D_MAIN);

					waitfor(DelayMs(PERIOD));
				} while (--n);

				if (params.flags){
					main -= background;
				}

	         switch (accumulation)
	         {
	           case ACCM_8:
	               ref_temp[k] = (uint16_t)(ref >> 3);
                  main_data[i * prm->num_of_em_stp + k] = (uint16_t)(main >> 3);
	           break;
	           case ACCM_32:
	               ref_temp[k] = (uint16_t)(ref >> 5);
                  main_data[i * prm->num_of_em_stp + k] = (uint16_t)(main >> 5);
	           break;
	           case ACCM_64:
	               ref_temp[k] = (uint16_t)(ref >> 6);
                  main_data[i * prm->num_of_em_stp + k] = (uint16_t)(main >> 6);
	           break;
	           case ACCM_128:
	               ref_temp[k] = (uint16_t)(ref >> 7);
                  main_data[i * prm->num_of_em_stp + k] = (uint16_t)(main >> 7);
	           default: ;
	         }

				em_target_pos += prm->em_stp_size;
	#ifdef   PRINTABLE_MO
				printf("Data for emission %d position\n", em_target_pos);
				printf("Ref data %d = %d \t main data %d = %d.\n", k, ref_temp[k], k,
						main_data[i * prm->num_of_em_stp + k]);
	#endif
				if (em_target_pos < MAX_MOT_POSITION){
					goToPos(SECOND, em_target_pos);
					waitfor (isPositionSet(SECOND, em_target_pos));
				}
				else
					break;

			}
				BitWrPortI(PADR, &PADRShadow, 0, EN_LAMP);
				num_of_steps_exec += k;
	//Return to home position
	/*
	//For parallelogram scanning
			  em_target_pos = em_initial_pos + em_stp_size;
			  em_initial_pos = em_target_pos;
	*/
	//For restangular scanning
			em_target_pos = initial_pos;
			goToPos(SECOND, em_target_pos);
			waitfor (isPositionSet(SECOND, em_target_pos));

			ref = 0;
			for (k = 0; k < prm->num_of_em_stp; ++k){
					ref += ref_temp[k];
					ref_data[i] = (uint16_t)(ref / prm->num_of_em_stp);
				}
	#ifdef   PRINTABLE
			printf("Data for excitation %d position\t", ex_target_pos);
			printf("Ref data %d = %u (%4x).\n", i, ref_data[i], ref_data[i]);
	#endif
			ex_target_pos += prm->ex_stp_size;
			if (ex_target_pos < MAX_MOT_POSITION){
				goToPos(FIRST, ex_target_pos);
				waitfor (isPositionSet(FIRST, ex_target_pos));
			}
			else
				break;
		}
	}
	return 1;
}

void readADC ()
{
   char i, serviceByte;

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

      BitWrPortI(PBDR, &PBDRShadow, 0, AD_CLK);
   }

   BitWrPortI(PADR, &PADRShadow, 1, AD_CS);
}

unsigned int getData(char chann)
{
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

char isPositionSet(unsigned int MotorAddr, unsigned int position)
{
	char i;
  	char status[8];

	int actual_pos;

   enum {false, true};

   readMotorStatus(GETFULLSTATUS2, MotorAddr, status);

#ifdef   PRINTABLE_MO
	printf ("Status 2 of TMC motor %d:\n", MotorAddr);
	for (i = 0; i < 8; ++i){
     	printf ("Byte %d = %2x.\n", i, status[i]);
   }
#endif

   actual_pos = ((int)status[1])<< 8 | (int)status[2];
	if (actual_pos == position)
	  return true;
	else
  	  return false;
}

