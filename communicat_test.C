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
//#use "RS232.lib"

#define PRINTABLE		1

#define CINBUFSIZE  15
#define COUTBUFSIZE 15

#define Esc 27

#define MSSG_TMOUT 3000UL
#define IDLE_TMOUT 5000UL

#define BUFF_SIZE 20

typedef struct {
	short d;
	float f[BUFF_SIZE];
} DATA;

const float ADC_LSB_VAL = 0.000076;
unsigned int ranSeed;				// Current Random Seed

unsigned int irand ()
{
	if (ranSeed == 0x5555) ranSeed--;
		ranSeed = (ranSeed << 1) + (((ranSeed>>15)^(ranSeed>>1)^ranSeed^1) & 1);

   return ranSeed;
}


unsigned int random (unsigned int range, unsigned int minimum )
{
	return irand() % range + minimum;
}

void main()
{
	int n, maxSize;

// 	char c[32];
   DATA data;
//   unsigned long t;

//	for (; ;)
//	{
#ifdef PRINTABLE
		printf("Start transmit ch.\n");
#endif

   maxSize = sizeof(DATA);

   for (n = 0; n < BUFF_SIZE; ++n)
   {
      data.f[n] = ADC_LSB_VAL * random(BUFF_SIZE, 2000);
      printf(" F[%d] = %f (%lx).\n", n, data.f[n], data.f[n]);
   }
	n = sizeof(data.f);
   data.d = 0;

   serCopen(115200);

//   t = MS_TIMER;
//   while ((n != 1) || ((char)data.d != Esc)) {
   while ((char)data.d != Esc) {
   /*
	//    serCputc('H');
   	 	if ((n = serCread(&data, maxSize, MSSG_TMOUT)) > 0) {
            if (n != maxSize) {
   				sprintf (c, " %d characters read ", n);
	      		serCputs(c);
   	   	}
*/
     	if ((data.d = serCgetc()) != -1 && data.d != Esc) {
      		serCwrite(&data.f, n);
//      		t = MS_TIMER;                    // start anew for next message
      	}
/*
      	else
      	{
	         if (MS_TIMER > t + IDLE_TMOUT)
	         {
	            serCputs(" Timed out! ");
	            t = MS_TIMER;                 // start anew for next message
	         }
	      }
/*
	         serCputc(c);
	         if ( c == '\r' ) {       // Cook ENTER into CR_LF
	            serCputc('\n');
	         }
	      }
*/
	   }
	   serCputs("Done\r\n");
	   while (serCwrFree() != COUTBUFSIZE);      // allow transmission to complete before closing
	   serCclose();
//	}
}