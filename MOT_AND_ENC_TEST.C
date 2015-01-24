/*================================================================*
 *Written by: 	Prokopov L. LDI
 *Date:			11.04.2006	Ver. 1.00
 *File:			Narciss_mot_TEST.c
 *For :			RMC2300
 *Project:     Narciss
 *Status:      developing
 *================================================================*/
//#memmap	xmem
#use "user.lib"
#use "TMC222.lib"

#define CINBUFSIZE  15
#define COUTBUFSIZE 15

#define	OUT		0x8000 	// for PE4
#define	IN			0xe000	// PE7 17.06
#define DUMM		0xA000	// for read encoders

// Output bits defenition
// Port A
#define AD_CS		0
#define EN_LAMP	1

//#define DAC_CS		3
#define ENC_NOE1		5				//encoder OE L.P. 14.12.2014 was 4

#define ENC_NOE2		6				//select encoders 0 or 1

// Port B
#define D_MAIN		1				//input, data from PMT
#define D_REF		5               //input, data from referenc channel
#define AD_CLK		7				//ADC serial clock

// Port D
#define I2CSCLBit 4
#define I2CSDABit 5
#define i2cRetries 100

#use "I2C_DEVICES.LIB"
#define DAC_ADDRESS 0x2c

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

#define	MAX_MS_STEP	32000	//max mickrosteps for new motor
#define  MAX_POINTS	500
#define  POS_STEP 	200

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

enum mot {FIRST,
			SECOND = 2};

char n_Status[9];
char motorAddr, OUTShadow;

const char mot_param[] = {((IRUN << 4)|IHOLD), ((VMAX << 4)|VMIN),
//									ACC, 0x00, 0x00};//halfstep
//									ACC, 0x00, 0x04};//1/4step
									ACC, 0x00, 0x0c};//16 microsteps
const char home_pos[] = {0x80, 0x00};
const char end_pos[]  = {0x7f, 0xff};	//8000
const char beg_pos[]  = {0xff, 0x05};

// port init function
void initPorts ();
void usDelay(int iDelay);
void MsDelay(int iDelay);
// Functions for TMC222
int send_TMC_Command (char addr, char cmnd);
int send_TMC_Param (char addr, char cmnd, char *par, char numb);
int read_TMC (char addr, char *buf);
int goToPos (char mot, int pos);
void readMotorStatus(char status, char mot);
int ReadEncoder(char mot);

//////////////////////////////////////////////////////////////////////////////
void main()
{
	char input, start, end, temp_pos[2];
    int target_pos, current_pos, ind, endInd;
    int enc_data[MAX_POINTS];


	initPorts ();
	i2c_init();

/*
    //input test
    while(1){
        if (BitRdPortE(IN, SW_HOME)){
			BitWrPortE(OUT, &OUTShadow, 1, TEST);
        }
        else {
			BitWrPortE(OUT, &OUTShadow, 0, TEST);
        }
    }
*/

	for (; ;){
//*
	    motorAddr = FIRST;
	    readMotorStatus(GETFULLSTATUS1, motorAddr);
	    send_TMC_Command((TMC_ADDR | motorAddr), RESETPOS);
	    readMotorStatus(GETFULLSTATUS2, motorAddr);
	    //write motor parameters
	    send_TMC_Param ((TMC_ADDR | motorAddr), SETMOTORPARAM, &mot_param[0], sizeof(mot_param));
	    readMotorStatus(GETFULLSTATUS1, motorAddr);

	    //motor to home position
	    printf("Mot %d go to home position \n", motorAddr);
	    send_TMC_Param ((TMC_ADDR | motorAddr), SETPOSITION, &home_pos[0], sizeof(home_pos));
	    //send_TMC_Command((TMC_ADDR | motorAddr), GETFULLSTATUS1);
	    while(BitRdPortE(IN, EM_SW_HOME)){
	        //readMotorStatus(GETFULLSTATUS2, motorAddr);
	        //read_TMC ((TMC_ADDR | motorAddr | RD_BIT), &n_Status[0]);
	        ;
	    }
	    send_TMC_Command((TMC_ADDR | motorAddr), HARDSTOP);
	    input = RdPortE(IN);
	    printf("Motor %d in home_pos (input = %x)\n", motorAddr, input);

	    readMotorStatus(GETFULLSTATUS1, motorAddr);
	    readMotorStatus(GETFULLSTATUS2, motorAddr);
	    send_TMC_Command((TMC_ADDR | motorAddr), RESETPOS);

      //Reset encoder
	    BitWrPortE(OUT, &OUTShadow, 0, EM_ENC_NRES);
	    BitWrPortE(OUT, &OUTShadow, 1, EM_ENC_NRES);

	    readMotorStatus(GETFULLSTATUS2, motorAddr);
       enc_data[0] = ReadEncoder(motorAddr);
       printf("First encoder after reset %6d \n", enc_data[0]);

	    printf("Mot %d go to End position \n", motorAddr);
	    //target_pos = ((int)end_pos[2]<< 8) + end_pos[3];
       ind = 0;
       target_pos = 0;
        //send_TMC_Param ((TMC_ADDR | motorAddr), SETPOSITION, &end_pos[0], 4);

	   while(BitRdPortE(IN, EM_SW_END)){
        	///
           target_pos += POS_STEP;
           temp_pos[0] = (char)(target_pos >> 8);
           temp_pos[1] = (char)target_pos;

           send_TMC_Param ((TMC_ADDR | motorAddr), SETPOSITION, &temp_pos[0], sizeof(temp_pos));
           MsDelay(100);
           enc_data[ind++] = ReadEncoder(motorAddr);
            /*
	        send_TMC_Command((TMC_ADDR | motorAddr), GETFULLSTATUS2);
	        read_TMC ((TMC_ADDR | motorAddr | RD_BIT), &n_Status[0]);
	        current_pos = ((int)n_Status[1] << 8) + n_Status[2];
	        if (current_pos == target_pos){
	            break;
	        }
            */
            //;
	   }

	   send_TMC_Command((TMC_ADDR | motorAddr), HARDSTOP);
      endInd = ind;

	   input = RdPortE(IN);
	   printf("Motor %d in end_pos (input = %x)\n", motorAddr, input);

	   readMotorStatus(GETFULLSTATUS1, motorAddr);
	   readMotorStatus(GETFULLSTATUS2, motorAddr);

      for (ind = 0; ind < endInd; ind++){
       	printf("Encoder %d = %6d\n", motorAddr, enc_data[ind]);
		}
//	*/
	//
	   motorAddr = SECOND;
	   readMotorStatus(GETFULLSTATUS1, motorAddr);
	   send_TMC_Command((TMC_ADDR | motorAddr), RESETPOS);
	   readMotorStatus(GETFULLSTATUS2, motorAddr);
//write motor parameters
		send_TMC_Param ((TMC_ADDR | motorAddr), SETMOTORPARAM, &mot_param[0], sizeof(mot_param));
	   readMotorStatus(GETFULLSTATUS1, motorAddr);
//motor to home position
	   printf("Motor %d go to home position \n", motorAddr);
	   send_TMC_Param ((TMC_ADDR | motorAddr), SETPOSITION, &home_pos[0], sizeof(home_pos));
//send_TMC_Command((TMC_ADDR | motorAddr), GETFULLSTATUS1);

	   while(BitRdPortE(IN, EX_SW_HOME)){
	        //readMotorStatus(GETFULLSTATUS2, motorAddr);
	        //read_TMC ((TMC_ADDR | motorAddr | RD_BIT), &n_Status[0]);
	       ;
	   }

	   send_TMC_Command((TMC_ADDR | motorAddr), HARDSTOP);
	   input = RdPortE(IN);
	   printf("Motor %d in home_pos (input = %x)\n", motorAddr, input);

	   readMotorStatus(GETFULLSTATUS1, motorAddr);
	   readMotorStatus(GETFULLSTATUS2, motorAddr);
	   send_TMC_Command((TMC_ADDR | motorAddr), RESETPOS);
      //Reset encoder
	   BitWrPortE(OUT, &OUTShadow, 0, EX_ENC_NRES);
	   BitWrPortE(OUT, &OUTShadow, 1, EX_ENC_NRES);

      readMotorStatus(GETFULLSTATUS2, motorAddr);
      enc_data[0] = ReadEncoder(motorAddr);
      printf("Second encoder after reset %6d \n", enc_data[0]);

	   printf("Motor %d go to End position \n", motorAddr);
	    //target_pos = ((int)end_pos[2]<< 8) + end_pos[3];
	    //send_TMC_Param ((TMC_ADDR | motorAddr), SETPOSITION, &end_pos[0], 4);
      ind = 0;
      target_pos = 0;

	   while(BitRdPortE(IN, EX_SW_END)){
        	target_pos += POS_STEP;
      	temp_pos[0] = (char)(target_pos >> 8);
         temp_pos[1] = (char)target_pos;

         send_TMC_Param ((TMC_ADDR | motorAddr), SETPOSITION, &temp_pos[0], sizeof(temp_pos));
         MsDelay(100);
			enc_data[ind++] = ReadEncoder(motorAddr);
        	/*
	        send_TMC_Command((TMC_ADDR | motorAddr), GETFULLSTATUS2);
	        read_TMC ((TMC_ADDR | motorAddr | RD_BIT), &n_Status[0]);
	        current_pos = ((int)n_Status[1] << 8) + n_Status[2];
	        if (current_pos == target_pos){
	            break;
	        }
            */
	   }

	   send_TMC_Command((TMC_ADDR | motorAddr), HARDSTOP);

      endInd = ind;
	   input = RdPortE(IN);
	   printf("Motor %d in end_pos (input = %x)\n", motorAddr, input);
	   readMotorStatus(GETFULLSTATUS1, motorAddr);
	   readMotorStatus(GETFULLSTATUS2, motorAddr);

      for (ind = 0; ind < endInd; ind++){
        	printf("Encoder %d = %6d\n", motorAddr, enc_data[ind]);
      }
	}	//end endless loop
}
//////////////////////////////////////////////////////////////////////////////
typedef union {
	struct
	{
		char	LSB;
		char	MSB;
	}by;
	int	val;
}EncVal;

int ReadEncoder(char mot)
{
	EncVal	enc;
    //char enc_byte;
	char newPADRShadow;
    //long temp_data;

	newPADRShadow = PADRShadow;
	//newPADRShadow &= ~(LAMP_ENABLE_MASK);			// LAMP_ENABLE = LOW; ->Disable lamp flash
//   res(&newPADRShadow, EN_LAMP);
	//newPADRShadow |= (SIP_MASK | E0_MASK);		// SIP = HIGH, E0 = HIGH;
   if(FIRST == mot){
    	res(&newPADRShadow, ENC_NOE1);
      set(&newPADRShadow, ENC_NOE2);
   }
   else {
    	set(&newPADRShadow, ENC_NOE1);
      res(&newPADRShadow, ENC_NOE2);
	}
   //select MSB byte
   BitWrPortE(OUT, &OUTShadow, 0, SEL );
   // Fix encoder value
	WrPortI(PADR, NULL, newPADRShadow);

	// Read Excitation encoder
	enc.by.MSB = RdPortE(DUMM);
//   printf("%x\t", enc.by.MSB);
	BitWrPortE(OUT, &OUTShadow, 1, SEL );		// 2 nd byte addr
	enc.by.LSB = RdPortE(DUMM);
//   printf("%x\n", enc.by.LSB);

	// Restore PADR
	WrPortI(PADR, &PADRShadow, PADRShadow);
//   printf("enc value = %6d (%x).\n" , enc.val, enc.val);
    return enc.val;
}