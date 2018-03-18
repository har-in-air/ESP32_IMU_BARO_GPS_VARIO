#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <errno.h>
#include <linux/input.h>	
#include <ncurses.h>

#define UBX    0x01
#define NAV_PVT 0x07

#define NMEA 0xF0
#define GGA 0
#define GLL 1
#define GSA 2
#define GSV 3
#define RMC 4
#define VTG 5

void ubxchecksum(uint8_t* buffer, int numbytes, uint8_t* pcka, uint8_t* pckb) {
   uint8_t cka, ckb;
   cka = ckb = 0;
   for (int inx = 0; inx < numbytes; inx++) {
      cka += buffer[inx];
      ckb += cka;
      }
   *pcka = cka;
   *pckb = ckb;
   }

uint8_t buffer[] = {0xB5,0x62,0x06,0x01,0x08,0x00,0x99,0x99,0,0,0,0,0,0,0xff,0xff};

void disableMessage(int fd, int class, int id) {
   buffer[6] = class;
   buffer[7] = id;
   buffer[9] = 0;   
   uint8_t cka, ckb;
   ubxchecksum(&buffer[2], 12, &cka, &ckb);
   buffer[14] = cka;
   buffer[15] = ckb;
   int res = write(fd,buffer,16);
   //printf("write returns %d\n", res);
   }

void enableMessage(int fd, int class, int id) {
   buffer[6] = class;
   buffer[7] = id;
   buffer[9] = 1;   
   uint8_t cka, ckb;
   ubxchecksum(&buffer[2], 12, &cka, &ckb);
   buffer[14] = cka;
   buffer[15] = ckb;
   int res = write(fd,buffer,16);
   //printf("write returns %d\n", res);
   }

int main(void)    	{
      WINDOW* w;
      w = initscr();
      timeout(50);

      int fd;
      fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);	
      if(fd == -1)	{
            printf("\n  Error! in Opening ttyUSB0  ");
            return -1;
            }
		
		struct termios SerialPortSettings;	/* Create the structure                          */

		tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */
		cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
		cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */
		SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
		SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
		SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
		SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */		
		SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
		SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
		SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
		SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 1 characters */
		//SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */

		if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) {
		    printf("\n  ERROR ! in Setting attributes");
         return -2;
         }
		tcflush(fd, TCIFLUSH); 
		char read_buffer[200];
		int  bytes_read = 0;  
 		int i = 0;


      while (1) {
		   bytes_read = read(fd,&read_buffer,200); 
		   if (bytes_read) {
      		   for(i=0;i<bytes_read;i++){	 
		            printf("%c",read_buffer[i]);
                  }
               }

         int key = getch();
         switch (key) {
            case 'a' : disableMessage(fd, NMEA, GLL);
            break;
            case 'A' : enableMessage(fd, NMEA, GLL);
            break;
            case 'b' : disableMessage(fd,NMEA, RMC);
            break;
            case 'B' : enableMessage(fd, NMEA, RMC);
            break;
            case 'c' : disableMessage(fd,NMEA, VTG);
            break;
            case 'C' : enableMessage(fd,NMEA, VTG);
            break;
            case 'd' : disableMessage(fd,NMEA,GSV);
            break;
            case 'D' : enableMessage(fd,NMEA,GSV);
            break;
            case 'e' : disableMessage(fd,NMEA,GSA);
            break;
            case 'E' : enableMessage(fd,NMEA,GSA);
            break;
            case 'f' : disableMessage(fd,NMEA,GGA);
            break;
            case 'F' : enableMessage(fd,NMEA,GGA);
            break;
            case 'g' : disableMessage(fd,UBX,NAV_PVT);
            break;
            case 'G' : enableMessage(fd,UBX,NAV_PVT);
            break;
            default: break;
            }
      }
		close(fd);
      return 0;
    	}

