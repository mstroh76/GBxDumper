// by Martin Strohmayer
// Licence: CC BY 4.0
// Compile: gcc gbxdumper.c -o gbxdumper -Wall -lwiringPi

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <ctype.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <limits.h>

#define MAX_PATH 4096

static sig_atomic_t end = 0;

static void sighandler(int signo) {
  end = 1;
}

typedef unsigned long DWORD;
typedef unsigned short WORD;
typedef unsigned char BYTE;
#define CBYTE const BYTE
#define CWORD const WORD
#define CDWORD const DWORD

enum CONTROLBIT{
	CONTROL_WR  = 1,
	CONTROL_RD  = 2,
	CONTROL_CS  = 4,
	CONTROL_CS2 = 8,
};

CBYTE ControlByteDefault = 0x0F; //RD,WR,CS,CS2
BYTE ControlByte = 0x0F;
CBYTE MCP_IOCON     = 0x0B; //IOCON.Bank 0
CBYTE MCP_Direction = 0x00;
CBYTE MCP_Write     = 0x14; //IOCON.Bank 0
CBYTE MCP_PullUp    = 0x0C; //IOCON.Bank 0
CBYTE MCP_Read      = 0x12; //IOCON.Bank 0
CBYTE MCP_PORTA     = 0x0; //IOCON.Bank 0
CBYTE MCP_PORTB     = 0x1; //IOCON.Bank 0


CBYTE MCP_Direction_B1 = 0x00; //IOCON.Bank 1
CBYTE MCP_Write_B1  = 0x0A; //IOCON.Bank 1
CBYTE MCP_Read_B1   = 0x09; //IOCON.Bank 1
CBYTE MCP_PORTA_B1  = 0x00; //IOCON.Bank 1
CBYTE MCP_PORTB_B1  = 0x10; //IOCON.Bank 1


CBYTE MCP_OUTPUT = 0x00;
CWORD MCP_WOUTPUT= 0x0000;
CBYTE MCP_INPUT  = 0xFF;
CWORD MCP_WINPUT = 0xFFFF;
CBYTE MCP_BANK0  = 0x00;
CBYTE MCP_BANK1  = 0x80;


DWORD GBA_Low_Address_A_Mask = 0x000000FF; //24 Bit -> Bit 0-7
DWORD GBA_Low_Address_B_Mask = 0x0000FF00; //24 Bit -> Bit 8-15
DWORD GBA_Low_Address_Mask   = 0x0000FFFF; //24 Bit -> Bit 0-15
DWORD GBA_High_Address_Mask  = 0x00FF0000; //24 Bit -> Bit 16-23

   static const unsigned char revtable[] = {
        0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
        0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
        0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
        0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
        0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
        0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
        0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
        0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
        0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
        0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
        0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
        0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
        0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
        0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
        0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
        0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
        0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
        0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
        0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
        0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
        0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
        0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
        0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
        0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
        0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
        0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
        0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
        0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
        0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
        0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
        0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
        0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
    };


   static const unsigned char Logo[] = {
	0x24, 0xFF, 0xAE, 0x51, 0x69, 0x9A, 0xA2, 0x21,
	0x3D, 0x84, 0x82, 0x0A, 0x84, 0xE4, 0x09, 0xAD,
	0x11, 0x24, 0x8B, 0x98, 0xC0, 0x81, 0x7F, 0x21,
	0xA3, 0x52, 0xBE, 0x19, 0x93, 0x09, 0xCE, 0x20,
	0x10, 0x46, 0x4A, 0x4A, 0xF8, 0x27, 0x31, 0xEC,
	0x58, 0xC7, 0xE8, 0x33, 0x82, 0xE3, 0xCE, 0xBF,
	0x85, 0xF4, 0xDF, 0x94, 0xCE, 0x4B, 0x09, 0xC1,
	0x94, 0x56, 0x8A, 0xC0, 0x13, 0x72, 0xA7, 0xFC,
	0x9F, 0x84, 0x4D, 0x73, 0xA3, 0xCA, 0x9A, 0x61,
	0x58, 0x97, 0xA3, 0x27, 0xFC, 0x03, 0x98, 0x76,
	0x23, 0x1D, 0xC7, 0x61, 0x03, 0x04, 0xAE, 0x56,
	0xBF, 0x38, 0x84, 0x00, 0x40, 0xA7, 0x0E, 0xFD,
	0xFF, 0x52, 0xFE, 0x03, 0x6F, 0x95, 0x30, 0xF1,
	0x97, 0xFB, 0xC0, 0x85, 0x60, 0xD6, 0x80, 0x25,
	0xA9, 0x63, 0xBE, 0x03, 0x01, 0x4E, 0x38, 0xE2,
	0xF9, 0xA2, 0x34, 0xFF, 0xBB, 0x3E, 0x03, 0x44,
	0x78, 0x00, 0x90, 0xCB, 0x88, 0x11, 0x3A, 0x94,
	0x65, 0xC0, 0x7C, 0x63, 0x87, 0xF0, 0x3C, 0xAF,
	0xD6, 0x25, 0xE4, 0x8B, 0x38, 0x0A, 0xAC, 0x72,
	0x21, 0xD4, 0xF8, 0x07 };




	//Settings (default)
	int LEDState = LOW;
	int GPIO_SW = 16;
	int GPIO_LED = 4;
	int GPIO_RD = 26;
	int bAutoAddressMode = 1;
	int bRDviaGIOMode = 1;
	int bVerify = 0;
	int bLog = 0;
	int I2CNo = 1;
	unsigned int SlaveAddr_IC1 = 0x22;
	unsigned int SlaveAddr_IC2 = 0x21;
	int bAD0_7_swap = 0;  //prototyp 1.0 set to 1!
	int bAD8_15_swap = 0;
	int bAD16_23_swap = 0;
	int bAD0_7_AD8_15_swap = 0;
	DWORD Force_GBA_MaxAddress = 0;
	char szFileDestination[PATH_MAX+1];
	DWORD offset = 0;


int I2CWriteValue(int fd, unsigned char Value) {
	if (write(fd, &Value, 1) != 1) {
		fprintf(stderr,
		"Failed to write byte to the i2c bus (%s)\n", strerror(errno));
		return 0;
	}
	return 1;
}

int I2CWrite(int fd, unsigned char Register, unsigned char Value) {
	unsigned char buf[2];

	buf[0] = Register;
	buf[1] = Value ;
	if (write(fd, buf, 2) != 2) {
		fprintf(stderr,
		"Failed to write byte to the i2c bus (%s)\n", strerror(errno));
		return 0;
	}
	return 1;
}

int I2CWriteWord(int fd, unsigned char Register, unsigned short Value) {
	unsigned char buf[3];

	buf[0] = Register;
	memcpy(&buf[1], &Value, 2);
	if (write(fd, buf, 3) != 3) {
		fprintf(stderr,
			"Failed to write word to the i2c bus (%s)\n", strerror(errno));
			return 0;
		}
		return 2;
	}

	int I2CRead(int fd, unsigned char Register, unsigned char* Value) {
		unsigned char Value1 = 0;
		if (write(fd, &Register, 1) != 1) {
			fprintf(stderr,
			"Failed to write to the i2c bus (%s)\n", strerror(errno));
			return 0;
		}

		if (read(fd, &Value, 1) != 1) {
			fprintf(stderr,
			"Failed to read byte from the i2c bus (%s)\n",strerror(errno));
			return 0;
		}
		*Value = Value1;
		return 1;
	}


	int I2CReadWord(int fd, unsigned char Register, unsigned short* Value) {
		unsigned short Value1 = 0;
		if (write(fd, &Register, 1) != 1) {
			fprintf(stderr,
			"Failed to write to the i2c bus (%s)\n", strerror(errno));
			return 0;
		}
		if (read(fd, &Value1, 2) != 2) {
			fprintf(stderr,
			"Failed to read word from the i2c bus (%s)\n",strerror(errno));
			return 0;
		}

		*Value = Value1;
		return 1;
	}

	WORD DumpBuffer[0x8000];
	BYTE* pDumpBuffer =  (BYTE*) DumpBuffer;

	WORD GBA_Last_LowAddress = 0;
	BYTE GBA_Last_LowAddress_A = 0;
	BYTE GBA_Last_LowAddress_B = 0;
	BYTE GBA_Last_HighAddress = 0;

	void SetAddress(int fd_X, int fd_Y, DWORD dwAddress, int bLog) {
		WORD GBA_LowAddress = dwAddress & GBA_Low_Address_Mask;
		BYTE GBA_LowAddress_A = dwAddress & GBA_Low_Address_A_Mask;
		BYTE GBA_LowAddress_B = (dwAddress & GBA_Low_Address_B_Mask) >> 8;
		BYTE GBA_HighAddress = (dwAddress & GBA_High_Address_Mask) >> 16;

		if (GBA_LowAddress != GBA_Last_LowAddress) {
			if (GBA_Last_LowAddress_A !=  GBA_LowAddress_A) {
				 int bAD0_7_swap2		 = (bAD0_7_AD8_15_swap)  ? bAD8_15_swap : bAD0_7_swap;
				unsigned char value = (unsigned int) bAD0_7_swap2 ? revtable[GBA_LowAddress_A] : GBA_LowAddress_A;
				if (bLog) printf("Address %X: Low Address Low BYTE=%X\n", (unsigned int) dwAddress, (unsigned int) value);
				if (bAD0_7_AD8_15_swap) {
					I2CWrite(fd_X, MCP_Write+MCP_PORTB, value);
				} else {
					I2CWrite(fd_X, MCP_Write+MCP_PORTA, value);
				}
				
			}
			if (GBA_Last_LowAddress_B !=  GBA_LowAddress_B) {

				 int bAD8_15_swap2		 = (bAD0_7_AD8_15_swap)  ? bAD0_7_swap : bAD8_15_swap;
				unsigned char value = (unsigned int) bAD8_15_swap2 ? revtable[GBA_LowAddress_B] : GBA_LowAddress_B;
				if (bLog) printf("Address %X: Low Address High BYTE=%X\n", (unsigned int) dwAddress, (unsigned int) value);
				if (bAD0_7_AD8_15_swap) {
					I2CWrite(fd_X, MCP_Write+MCP_PORTA, value);
				} else {
					I2CWrite(fd_X, MCP_Write+MCP_PORTB, value);
				}
			}
			GBA_Last_LowAddress = GBA_LowAddress;
			GBA_Last_LowAddress_A = GBA_LowAddress_A;
			GBA_Last_LowAddress_B = GBA_LowAddress_B;
		}
		if (GBA_HighAddress != GBA_Last_HighAddress) {
			unsigned char value = (unsigned int) bAD16_23_swap ? revtable[GBA_HighAddress] : GBA_HighAddress;
			if (bLog) printf("Address %X: High Address BYTE=%X\n", (unsigned int) dwAddress, (unsigned int) value);
			I2CWrite(fd_Y, MCP_Write + MCP_PORTA, value);
			GBA_Last_HighAddress = GBA_HighAddress;
		}
	}

	int GetData(int fd_X, DWORD dwAddress, WORD *wData, int bLog) {
		unsigned char* WordByteArray = (unsigned char*)wData;

		if (!I2CReadWord(fd_X, MCP_Read + MCP_PORTA, wData)) {
			return 0;
		}
		unsigned char Byte0 = (unsigned int) bAD0_7_swap ? revtable[WordByteArray[0]] : WordByteArray[0];
		unsigned char Byte1 = (unsigned int) bAD8_15_swap ? revtable[WordByteArray[1]] : WordByteArray[1];
		if (bAD0_7_AD8_15_swap) {
			WordByteArray[0] = Byte1;
			WordByteArray[1] = Byte0;
		} else {
			WordByteArray[0] = Byte0;
			WordByteArray[1] = Byte1;			
		}

		if (bLog) printf("Address %X: Data=%X, byte=0x%02X%02X\n", (int)dwAddress, (int)*wData, (int)WordByteArray[0], (int)WordByteArray[1]);
		return 1;
	}


	void SetControlBit(int fd_Y, BYTE nBit) {
		BYTE nByte = ControlByte | nBit;
		if (I2CWrite(fd_Y, MCP_Write + MCP_PORTB, nByte) > 0) {
			ControlByte = nByte;
		}
	}

	void ResetControlBit(int fd_Y, BYTE nBit) {
		BYTE nByte = ControlByte & (~nBit);
		if (I2CWrite(fd_Y, MCP_Write + MCP_PORTB, nByte) > 0) {
			ControlByte = nByte;
		}
	}

	const char szGBARelaeseList[]= "gbalist.csv";

	 
	 const char* GetCVSTextValue(char** ppSrcBuffer, const char *pSrcBufferMax, char* szDesBuffer, int nDesMaxSize) {
		int nDesBufferPos = 0;
		const char*	 pDataPos = NULL;
		char* pSrcBuffer = ppSrcBuffer[0];
		
		while (pSrcBuffer[0]=='\t' && pSrcBuffer<pSrcBufferMax) { //goto next column
			pSrcBuffer++;
		}
		pDataPos = pSrcBuffer;
		while (pSrcBuffer[0]!='\t' && pSrcBuffer<pSrcBufferMax) {
			if (szDesBuffer && nDesBufferPos<nDesMaxSize) {
				szDesBuffer[nDesBufferPos] = pSrcBuffer[0];
				nDesBufferPos++;
			}
			pSrcBuffer = &pSrcBuffer[1];
		}
		if (szDesBuffer && nDesMaxSize>0) {
			szDesBuffer[nDesBufferPos] = '\0';
		}
		ppSrcBuffer[0] = pSrcBuffer;
		return pDataPos;
	 }
	 
	int GetCVSIntValue(char** pSrcBuffer, const char *pSrcBufferMax) {
		const char* pDataPos = GetCVSTextValue(pSrcBuffer, pSrcBufferMax , NULL, 0);
		
		if (pDataPos) {
			return atoi(pDataPos);
		} else {
			return 0;
		}
	}

	void print_usage() {
	    printf("Usage: \n");
		printf("  r ... read pin via I2C instead of GPIO (have to match with board jumper, slow)\n");
		printf("  g <Pin> ...  GPIO Pin for read operation\n");
		printf("  e <Pin> ...  GPIO Pin for LED (0=off)\n");
		printf("  s <Pin> ...  GPIO Pin for switch (0=off)\n");
		printf("  n ... Don't use auto adress mode (slow)\n");
		printf("  i ... I2C number\n");
		printf("  l <hexvalue> ... IC1 (AD0-AD15) I2C-Address (22=0x22)\n");	
		printf("  h <hexvalue> ... IC2 (A16-A23) I2C-Address (21=0x21)\n");
		printf("  f ... verify (need option n)\n");
		printf("  v ... verbose\n");
		printf("  a ... IC Bits AD0-7 swapped\n");
		printf("  b ... IC Bits AD8-15 swapped\n");
		printf("  c ... IC Bits AD16-23 swapped\n");
		printf("  x ... IC Byte AD0-7 and AD8-15 swapped\n");
		printf("  z ... force dump size (>=64 ... KB, <=32 ... MB)\n");	
		printf("  d <path> ... save dumped file to path (same drive)\n");
		printf("  o <value> ... offset reading (need option z)\n");
		printf("\n\n");
	}

	int main(int argc, char* argv[]) {
		int fd_X;
		int fd_Y;
		char dev_i2c[20];
		float fTimePerOperation;
		struct timeval t1, t2, t3;
		double elapsedTime;
		struct sigaction sa;
		char* GBARelaeseListBuffer = NULL;
		struct stat fileStat;
		FILE *fp;
		int c;

    //char szPWD[MAX_PATH];
   // chdir("/path/to/change/directory/to");
   // getcwd(cwd, sizeof(cwd));
  // strcpy(szPWD, argv[0]);
   // printf("Current working dir: %s\n", szPWD);

	int nReturn;
	nReturn = system("./start.sh");
	printf("system call for \"./start.sh\" returned %d (path)\n", nReturn);
//	exit(0);
/*
		//Settings (default)
		int LEDState = LOW;
		int GPIO_SW = 16;
		int GPIO_LED = 4;
		int GPIO_RD = 26;
		int bAutoAddressMode = 1;
		int bRDviaGIOMode = 1;
		int bVerify = 0;
		int bLog = 0;
		int I2CNo = 1;
		unsigned int SlaveAddr_IC1 = 0x22;
		unsigned int SlaveAddr_IC2 = 0x21;
		int bAD0_7_swap = 0;  //prototyp 1.0 set to 1!
 		int bAD8_15_swap = 0;
		int bAD16_23_swap = 0;
		int bAD0_7_AD8_15_swap = 0;
		DWORD Force_GBA_MaxAddress = 0;
*/
		int nValue;
		
		szFileDestination[0]='\0';
		while ((c = getopt (argc, argv, "g:nri:l:h:fve:s:abcxz:d:o:")) != -1) {
			switch (c) {
				case 's':  //GPIO for switch
					GPIO_SW = atoi(optarg);
					break;
				case 'e':  //GPIO for LED
					GPIO_LED = atoi(optarg);
					break;
				case 'g':  //GPIO for read operation
					GPIO_RD = atoi(optarg);
					break;
				case 'i':  //I2C Number
					I2CNo = atoi(optarg);
					break;				
				case 'l':  //IC1 I2C-Address
					sscanf(optarg, "%X", &SlaveAddr_IC1);
					break;				
				case 'h':  //IC2 I2C-Address
					sscanf(optarg, "%X", &SlaveAddr_IC2);
					break;		
				case 'n':
					bAutoAddressMode = 0;
					break;
				case 'r':
					bRDviaGIOMode = 0;
					break;
				case 'f':
					bVerify = 1;
					break;
				case 'v':
					bLog = 1;
					break;				
				case 'a':
					bAD0_7_swap = 1;
					break;				
				case 'b':
					bAD8_15_swap = 1;
					break;			
				case 'c':
					bAD16_23_swap = 1;
					break;		
				case 'x':
					bAD0_7_AD8_15_swap = 1;
					break;		
				case 'o':
					offset = (DWORD) atoi(optarg);
					break;		
				case 'z':
					nValue = atoi(optarg);
					if (nValue>=1 && nValue<=32) {
						Force_GBA_MaxAddress = nValue*1024*1024/2;
					} else 	if (nValue>=64 && nValue<=32768) {					
						Force_GBA_MaxAddress = nValue*1024/2;
					} else {
						printf("error invalid GBA size value %d\n", nValue);	
					}
					break;		
				case 'd':
					strncpy(szFileDestination, optarg, MAX_PATH-1);
					strncat(szFileDestination, "/", MAX_PATH);
					szFileDestination[MAX_PATH] = '\0';
					break;
				default:
					print_usage();
					exit(EXIT_FAILURE);
					break;
	       }	
	   }

		printf("Working paramters:\n");  
		printf("  - using LED GPIO %d\n", GPIO_LED);
		printf("  - using switch GPIO %d\n", GPIO_SW);
		printf("  - using I2C-%d\n", I2CNo);
		printf("  - using IC1 (AD0-AD15) I2C-Address 0x%02X\n", SlaveAddr_IC1);
		printf("  - using IC2 (A16-A23, Control) I2C-Adress 0x%02X\n", SlaveAddr_IC2);
		if (bAD0_7_swap) {
			printf("  - swapping IC1 bits of low byte (AD0-AD7)\n");
		}
		if (bAD8_15_swap) {
			printf("  - swapping IC1 bits of high byte (AD8-AD15)\n");
		}
		if (bAD0_7_AD8_15_swap) {
			printf("  - swapping IC1 low and high byte\n");
		}	
		if (bAD16_23_swap) {
			printf("  - swapping IC2 bits of byte (AD16-AD23)\n");
		}
		if (bRDviaGIOMode) {
			printf("  - using RD-Pin via GPIO %d (have to match with board jumper)\n", GPIO_RD);
		} else {
			printf("  - using RD-Pin via I2C (have to match with board jumper, slow)\n");
		}		
		if (bAutoAddressMode) {
			printf("  - using auto address mode\n");
		} else {
			printf("  - address is set explicit (slow)\n");
			if (bVerify) {
				printf("  - Verifing read operation\n");
			}
		}
		if (Force_GBA_MaxAddress) {
			printf("  - Force GBA dump size of %lu kB\n", Force_GBA_MaxAddress*2/1024);

			if (offset) {
				printf("  - using read offset %lu\n", offset);
			}

		}
		if(szFileDestination[0]!='\0') {
			printf("  - Save GBA dump file to directory '%s'\n", szFileDestination);
		}
		if (bLog) {
			printf("  - verbose\n");
		}
		printf(" \n");

		memset(&sa, 0, sizeof(struct sigaction));
		sa.sa_handler = sighandler;
		sigaction(SIGINT, &sa, NULL);
		sigaction(SIGQUIT, &sa, NULL);
		sigaction(SIGTERM, &sa, NULL);

		if (stat(szGBARelaeseList,&fileStat) >=0)  { 
			int nRead = 0;
			printf("loding gba release list (%d kB)...", (int)fileStat.st_size/1024); 
			fflush(stdout);
			GBARelaeseListBuffer = (char*)malloc(fileStat.st_size+1);
			if (GBARelaeseListBuffer) {
				fp = fopen(szGBARelaeseList, "r"); 
				if (fp) {
					nRead = fread(GBARelaeseListBuffer, sizeof(char), fileStat.st_size, fp);	
					printf("done\n");
				} else {
					printf("file error\n");
				}
				GBARelaeseListBuffer[nRead] = '\0';
				fflush(stdout);
			} else {
				printf("file buffer error\n");
			}
		} else {
			printf("file stat filed with error %d", errno);
		}

		printf("Init GPIO interface...\n	");
		if (wiringPiSetupGpio() == -1){
			printf("wiringPiSetup failed\n\n");
			exit(EXIT_FAILURE);
		}
		do {
		if (GPIO_LED) {
			printf("Set LED GPIO to on...\n	");
			pinMode(GPIO_LED, OUTPUT);
			LEDState = LOW;
			digitalWrite(GPIO_LED, LEDState);
		}
		if (GPIO_SW) {
			printf("Set switch GPIO to read... please press switch\n");
			pinMode(GPIO_SW, INPUT);
			pullUpDnControl(GPIO_SW, PUD_UP) ;
			do {
				usleep(250000);
			} while(digitalRead(GPIO_SW)==HIGH && !end);
		}
		if (end) {
			if (GPIO_LED) {
				printf("Set LED GPIO to input\n");
				pinMode(GPIO_LED, INPUT);
			}
			printf("Closing programm...\n");
			exit(0);
		}	
		
		printf("Set RD GPIO to default...\n	");
		pinMode(GPIO_RD, OUTPUT);
		digitalWrite(GPIO_RD, HIGH);

		sprintf(dev_i2c, "/dev/i2c-%d", I2CNo);
		printf("open dev_i2c '%s'...\n", dev_i2c);
		if ((fd_X = open(dev_i2c, O_RDWR)) < 0) {
			fprintf(stderr,"Failed to open i2c bus '%s'\n", dev_i2c);
			exit(12);
		}
		printf("set slave 0x%02X for IC1 (AD0-AD15) ...\n", SlaveAddr_IC1);
		if (ioctl(fd_X, I2C_SLAVE, SlaveAddr_IC1) < 0) {
			fprintf(stderr,
			"Failed to acquire i2c bus access or talk to slave %X\n", SlaveAddr_IC1);
			close(fd_X);
			exit(13);
		}

		printf("open dev_i2c '%s'...\n", dev_i2c);
		if ((fd_Y = open(dev_i2c, O_RDWR)) < 0) {
			fprintf(stderr,"Failed to open i2c bus '%s'\n", dev_i2c);
			close(fd_X);
			exit(22);
		}
		printf("set slave 0x%02X for IC2 (A16-A23, Control) ...\n", SlaveAddr_IC2);
		if (ioctl(fd_Y, I2C_SLAVE, SlaveAddr_IC2) < 0) {
			fprintf(stderr,
			"Failed to acquire i2c bus access or talk to slave %X\n", SlaveAddr_IC2);
			close(fd_X);
			close(fd_Y);
			exit(23);
		}

		printf("write direction IC1 Port A/B (AD0-AD16) to output, default ...\n");
		I2CWriteWord(fd_X, MCP_Write, 0x0000);
		I2CWriteWord(fd_X, MCP_Direction, MCP_WOUTPUT);

		printf("write direction IC2 Port A (A16-A23) to output, default  ...\n");
		I2CWrite(fd_Y, MCP_Write + MCP_PORTA, 0x00);
		I2CWrite(fd_Y, MCP_Direction + MCP_PORTA, MCP_OUTPUT);

		printf("write direction IC2 Port B (Control) to output, default  ...\n");
		I2CWrite(fd_Y, MCP_Write  + MCP_PORTB, ControlByte);
		I2CWrite(fd_Y, MCP_Direction + MCP_PORTB, MCP_OUTPUT);

		DWORD GBA_Address = 0x00000000;
		// 0x1000000 ... 32 MB
		// 0x0800000 ... 16 MB
		// 0x0400000 ...  8 MB
		// 0x0200000 ...  4 MB
		// 0x0080000 ...512 kB
		DWORD GBA_MaxAddress = 0x2000;   //  8 kB .. include header
		if (Force_GBA_MaxAddress) {
			GBA_MaxAddress = Force_GBA_MaxAddress;
		} else {
			offset = 0;
		}
			
	FILE* fpDumpFile = NULL;
	int bGetChar = 0;
	const char cszFilename[] = "game.gba";
	char szFinalName[12+4+1];

	szFinalName[0] = '\0';
	printf("Write dump to file '%s', ", cszFilename);
	fpDumpFile=fopen(cszFilename,"w+");
	if (fpDumpFile==NULL){
	   printf("Could not reate file, error %d!", errno);
	   exit(EXIT_FAILURE);
	}

	int LED_Duration=0;//ms
	unsigned int LED_Limit=0;
	int PercentFinished = 0;
	fTimePerOperation=0;
	gettimeofday(&t1, 0);
	for (GBA_Address = 0x00000000+offset; GBA_Address<GBA_MaxAddress; GBA_Address++) {
/*		if (GPIO_LED) {
			LED_BlockSize=GBA_MaxAddress/7;
			LED_Limit = 0x8000 >> (GBA_Address/LED_BlockSize); 
			if ((GBA_Address & LED_Limit) == 0 ) {
				if (LEDState!=LOW) {
					LEDState=LOW;
					digitalWrite(GPIO_LED, LEDState);
				}
			} else {
				if (LEDState!=HIGH) {
					LEDState=HIGH;
					digitalWrite(GPIO_LED, LEDState);
				}		
			}
		}
*/
		if (GPIO_LED) {
			PercentFinished = (10*GBA_Address)/GBA_MaxAddress;
			if (0==PercentFinished) {
				PercentFinished=1;
			}
			LED_Duration=3000-((3000*PercentFinished)/10);//ms
			if (fTimePerOperation!=0) {
				LED_Limit=LED_Duration*1000/fTimePerOperation;
			} else {
				LED_Limit=2000;
			}
			if (GBA_Address%LED_Limit>LED_Limit/2) {
				if (LEDState!=LOW) {
					LEDState=LOW;
					digitalWrite(GPIO_LED, LEDState);
				}
			} else {
				if (LEDState!=HIGH) {
					LEDState=HIGH;
					digitalWrite(GPIO_LED, LEDState);
				}		
			}
			
		}
		if (GBA_Address % 0x8000 == 0 || 0xB0 == GBA_Address) {
			if (0xB0 == GBA_Address) {
				char Name[12+1];
				char GameCode[4+1];
				char GameCodeEx[9+1];

				memset(Name,0,sizeof(Name));
				strncpy(Name, (char*)&pDumpBuffer[0xA0],12);
				snprintf(szFinalName,12+4+1, "%s.gba", Name);
				printf("\n-> Game  : %s (Save: %s)", Name, szFinalName); 

				memset(GameCode,0,sizeof(GameCode));
				strncpy(GameCode, (char*)&pDumpBuffer[0xAC],4);
				printf("\n-> GameCode  : %s", GameCode); 
				strcpy(GameCodeEx, "AGB-");
				strcat(GameCodeEx, GameCode);
				strcat(GameCodeEx, "-");

				printf("\n-> Header: ");
				if (!memcmp(Logo, &pDumpBuffer[0x04], sizeof(Logo)-4)) {
					printf("valid ");
				} else {
					printf("invalid ");
				}
				printf("\n-> Header Complement (checksum): ");
				WORD wChecksum = 0xFF00;
				int nAddress;
				for (nAddress=0xA0;  nAddress<=0xBC; nAddress++) {
					wChecksum-=pDumpBuffer[nAddress];
				}
				wChecksum -= 0x19;
				wChecksum &= 0x00FF;
				if (wChecksum == pDumpBuffer[0xBD]) {
					printf("valid (0x%x)",(unsigned int)wChecksum);
				} else {
					printf("invalid (calc 0x%X, found 0x%X)", (unsigned int)wChecksum, (unsigned int)pDumpBuffer[0xBD]);
					end = 1;
				}
				fflush(stdout);	
				char* pPos = NULL;
				if (GBARelaeseListBuffer) {
					pPos = strstr(GBARelaeseListBuffer, GameCodeEx);
				}
				if (pPos) {
					char szComplement[2+1];
					char szCRC32[8+1];
					char szMemoryType[20+1];
					char szGameCodeFull[12+1];
					const char* pSrcBufferMax = &GBARelaeseListBuffer[fileStat.st_size];
					
					GetCVSTextValue(&pPos, pSrcBufferMax, szGameCodeFull, 12);
					printf("\n-> Gamecode gbalist: %s ", szGameCodeFull);
					
					int nROMSizeMBit = GetCVSIntValue(&pPos, pSrcBufferMax);
					int nROMSizeMB = nROMSizeMBit / 8;
					if (!Force_GBA_MaxAddress) {
						GBA_MaxAddress = nROMSizeMB * 1024 * 1024 / 2; //16 bit per Address
					}
					printf("\n-> ROM Size: %d MBit (%d MByte, max dump address 0x%X)", nROMSizeMBit, nROMSizeMB, (int)GBA_MaxAddress);
					GetCVSTextValue(&pPos, pSrcBufferMax, szComplement, 2);
					printf("\n-> Complement: 0x%s", szComplement);
					
					GetCVSTextValue(&pPos, pSrcBufferMax, szCRC32, 8);
					printf("\n-> CRC32: 0x%s", szCRC32);

					GetCVSTextValue(&pPos, pSrcBufferMax, szMemoryType, 20);
					printf("\n-> RAM type: %s", szMemoryType);	

					int nRAMSizeKBit = GetCVSIntValue(&pPos, pSrcBufferMax);
					int nRAMSizeKB = nRAMSizeKBit / 8;
					int nRAMSizeByte = nRAMSizeKBit * 1024 / 8;
					if (nRAMSizeKBit>=8) {
						printf("\n-> RAM size: %d kBit (%d KByte)", nRAMSizeKBit, nRAMSizeKB);	
					} else {
						printf("\n-> RAM size: %d kBit (%d Byte)", nRAMSizeKBit, nRAMSizeByte);
					}									
				} else {
					printf("gamecode '%s' not found, type and size unknown", GameCodeEx);
					if (!Force_GBA_MaxAddress) {
						printf(", leaving programm (please set dump size)");	
						end = 1;
					} else {
						printf(", using dump size %lu kB", Force_GBA_MaxAddress*2/1024);	
					}
				}
				fflush(stdout);
			} else {
			}
			if(GBA_Address == 0x8000) {
				gettimeofday(&t2, 0);
			}
			if(GBA_Address == 0x10000) {
				gettimeofday(&t3, 0);
				elapsedTime = (t3.tv_sec - t2.tv_sec)  + (t3.tv_usec - t2.tv_usec)/1000000.0;
				fTimePerOperation = elapsedTime * 1000000.0f / 0x8000;
				printf("\n-> Speed: dumping %d kB took %.1f sec. (%.0f microsec. per operation), full dump will take %.1f min", 0x8000*2/1024, elapsedTime, fTimePerOperation, elapsedTime/0x8000*(GBA_MaxAddress-0x10000)/60);
			}
			int nDataBlock  = ((int)GBA_Address / 0x8000)*32*sizeof(WORD);
			printf("\n-> [%d0%%] %04d KB (%d KB per dot)", PercentFinished, nDataBlock,0x1000/0x400*sizeof(WORD)); 
			fflush(stdout);
		}
		if (!bAutoAddressMode || 0x00000000 == GBA_Address) {
			if (end) break;
			if (bLog) printf("-> Set CS,RD high\n");
			if (bGetChar) getchar();
			SetControlBit(fd_Y, CONTROL_CS | CONTROL_RD);//CS_High, RD_High
			if (bRDviaGIOMode) {
				digitalWrite(GPIO_RD, HIGH); // RD_High
			}
		} else {
			if (end) break;
			if (bLog) printf("-> Set RD high\n");
			if (bGetChar) getchar();
			if (bRDviaGIOMode) {
				digitalWrite(GPIO_RD, HIGH); // RD_High
				//usleep(1);
			} else {
				SetControlBit(fd_Y, CONTROL_RD);// RD_High
			}
		}
		if (!bAutoAddressMode || 0x00000000 == GBA_Address) {
			if (end) break;
			if (bLog) printf("-> set AD Port to output\n");
			if (bGetChar) getchar();
			I2CWriteWord(fd_X, MCP_Direction, MCP_WOUTPUT); //SetOutputDirection Address

			if (end) break;
			if (bLog) printf("-> Set address %X\n", (int)GBA_Address);
			if (bGetChar) getchar();
			SetAddress(fd_X, fd_Y, GBA_Address, bLog);

			if (end) break;
			if (bLog) printf("-> Set CS low\n");
			if (bGetChar) getchar();
			ResetControlBit(fd_Y, CONTROL_CS);//CS_Low

			if (end) break;
			if (bLog) printf("-> set AD Port to input\n");
			if (bGetChar) getchar();
			if (!I2CWriteWord(fd_X, MCP_Direction, MCP_WINPUT))
				break;
		}

		if (end) break;
		if (bLog) printf("-> set RD low\n");
		if (bGetChar) getchar();
		if (bRDviaGIOMode) {
			digitalWrite(GPIO_RD, LOW); // RD_Low
		} else {
			ResetControlBit(fd_Y, CONTROL_RD);
		}

		if (end) break;
		if (bLog) printf("-> get data\n");
		if (bGetChar) getchar();
		WORD wData,wData2;
		if (!GetData(fd_X, GBA_Address, &wData, bLog))
			break;
		if (bVerify & !bAutoAddressMode) {
			if (!GetData(fd_X, GBA_Address, &wData2, bLog))
				break;
			if (wData2 != wData) {
				printf("-> error %hu<>%hu, verify ...", wData, wData2);
				if (!GetData(fd_X, GBA_Address, &wData, bLog))
					break;
				printf(" finally use %hu\n", wData);
			}
		}
		DumpBuffer[GBA_Address % 0x8000] = wData;
		if (1 != fwrite(&wData, sizeof(wData), 1, fpDumpFile)) {
			printf("Error wirting to file %s\n", cszFilename);
			break;
		}
		if (GBA_Address % 0x1000 == 0) {
			//printf("F: %d0%%, D: %d, L: %d\n", PercentFinished, LED_Duration, LED_Limit );
			if (GPIO_SW && GBA_Address>0x100 && digitalRead(GPIO_SW)==LOW) {
				printf("\ncancel dumping\n");
				fflush(stdout);
			 	while (!end && digitalRead(GPIO_SW)==LOW){
					usleep(500000);
				} 
				break;
			} 
			
			printf(".");
			fflush(stdout);
		}
	}
	printf("\n-> Set control byte to default\n");
	SetControlBit(fd_Y, ControlByteDefault );//CS_High, RD_High
	I2CWriteWord(fd_X, MCP_Direction, MCP_WINPUT); //SetInputDirection Data
	if (bRDviaGIOMode) {
		digitalWrite(GPIO_RD, HIGH); // RD_HIGH
		pinMode(GPIO_RD, INPUT);
	}
	
	gettimeofday(&t2, 0);
	elapsedTime = (t2.tv_sec - t1.tv_sec)  + (t2.tv_usec - t1.tv_usec)/1000000.0;
	if (!end && GBA_Address==GBA_MaxAddress) {
		int Min, Sec;
		fTimePerOperation = elapsedTime * 1000000.0f / GBA_MaxAddress;
		Min = elapsedTime/60;
		Sec = (elapsedTime-60*Min)+0.5;
		printf("dumping %ld MB took %d min and %d sec (%g). (%.0f microsec. per operation)\n", GBA_MaxAddress*2/1024/1024, Min, Sec, elapsedTime, fTimePerOperation);
	}
	fflush(fpDumpFile);
	fclose(fpDumpFile);
	close(fd_X);
	close(fd_Y);
	if (!end && szFinalName[0]!='\0') {
		if (rename(cszFilename, szFinalName)!=0) {
			perror("Renaming GBA dump file failed");
		}
		strncat(szFileDestination, szFinalName, MAX_PATH);
		szFileDestination[MAX_PATH] = '\0';
	        nReturn = system("./prestore.sh");
        	printf("system call for \"./prestore.sh\" returned %d\n", nReturn);
	
		if (szFileDestination[0]!=0 && rename(szFinalName, szFileDestination)!=0) {
			perror("moving GBA dump file to destination failed");
			
		}
		char szPostStore[MAX_PATH];
		sprintf(szPostStore, "./poststore.sh \"%s\"", szFinalName);
		//nReturn = system("./poststore.sh");
		nReturn = system(szPostStore);
                printf("system call for \"./poststore.sh\" returned %d\n", nReturn);

	}
	}while(GPIO_SW && !end);
	if (GBARelaeseListBuffer) {
		free(GBARelaeseListBuffer);
		GBARelaeseListBuffer = NULL;
	}
	if (GPIO_LED) {
		printf("Set LED GPIO to input\n");
		pinMode(GPIO_LED, INPUT);
	}
	return 0;
}


