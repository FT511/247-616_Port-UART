/**
 * @file    SerialPort_write.c
 * 
 * @brief Serial Port Programming in C (Serial Port Write)  
 * Non Cannonical mode 
 * Sellecting the Serial port Number on Linux   
 * /dev/ttyUSBx - when using USB  to Serial Converter, where x can be 0,1,2...etc  
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc 
 * termios structure -  /usr/include/asm-generic/termbits.h  
 * use "man termios" to get more info about  termios structure 
 * @author  Kevin Cotton
 * @date    2024-08-02
 */
#define _GNU_SOURCE

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions
#include <sys/wait.h> 


const char* processusPereOuFils;
int fd; //File Descriptor
// device port série à utiliser 
//const char *portTTY = "/dev/ttyGS0"; 
const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter




/// @brief Code exécuté par le processus Père
/// @param  aucun
void codeDuProcessusParent(void)
{
 // Read data from serial port 
	tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
	char read_byte = 0;
	char  bytes_read = 0;    // Number of bytes read by the read() system call 
	int i = 0;

    while (1)
    {
        bytes_read = read(fd, &read_byte, 1); // Read the data 
        if(read_byte == '!')
        {   
            break;
        }
    	printf("%c", read_byte);
        read_byte = 0x00;
    }
}

/// @brief Code exécuté par le processus Fils
/// @param aucun
void codeDuProcessusEnfant(void)
{
    while(1)
    {
    // Write data to serial port 
    char write_byte = 0;
	int  bytes_written  = 0;  	// Value for storing the number of bytes written to the port 

    write_byte = getchar();
    if(write_byte == 'q')
    {
        break;
    }
	bytes_written = write(fd, &write_byte, sizeof(write_byte)); // use write() to send data to port 
										// "fd"                   - file descriptor pointing to the opened serial port
										//	"write_buffer"         - address of the buffer containing data
										// "sizeof(write_buffer)" - No of bytes to write 
    }
}


    
void InitPortSerie(void)
{
	
	printf("\n Ecriture Port Serie");

	// Opening the Serial Port 
	fd = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
								// O_RDWR Read/Write access to serial port           
								// O_NOCTTY - No terminal will control the process   
								// O_NDELAY -Non Blocking Mode,Does not care about-  
								// -the status of DCD line, Open() returns immediatly                                        
	if(fd == -1) // Error Checking
		printf("\n Erreur! ouverture de %s ", portTTY);
	else
		printf("\n Ouverture de %s reussit ", portTTY);

	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read  Speed   
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size
	SerialPortSettings.c_cflag |=  CS8;      //Set the data bits = 8
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines 

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);	// Disable XON/XOFF flow control both i/p and o/p

	SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  

    SerialPortSettings.c_lflag |= ICANON;  // Set the canonical mode 

	SerialPortSettings.c_oflag &= ~OPOST;	//No Output Processing

    // Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 1; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 0; // Wait 3sec (0 for indefinetly)

	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
		printf("\n  Erreur! configuration des attributs du port serie");

}


void main(void)
{
    InitPortSerie();

    pid_t pid;
    pid = fork();

    // Appel fonction Enfant
    if(pid == 0)
    {
        codeDuProcessusEnfant();
    }

    // Appel fonction Parent
    if(pid != 0)
    {
        codeDuProcessusParent();
        wait(NULL);
    }
    

    close(fd); // Close the serial port
}




