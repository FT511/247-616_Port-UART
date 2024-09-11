/**
 * @file    fork2_pipe2_uart.c
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
int pipeLirePortSerie[2];
int pipeEcrirePortSerie[2];

// device port série à utiliser 
//const char *portTTY = "/dev/ttyGS0"; 
const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter




/// @brief Code exécuté par le processus Père
/// @param  aucun
void codeDuProcessusParent(void)
{
    int n = 0;

    while(n < 10)
    {
        printf("1 faire quelques trucs .... \n");
        n++;
        sleep(3);
    }
}

/// @brief Code exécuté par le processus Fils
/// @param aucun
void codeDuProcessusEnfant(void)
{
    char buf;
    char read_byte = 0;
    char write_byte = 0;

    close(pipeLirePortSerie[1]);  // Fermer l'extrémité d'écriture
    close(pipeEcrirePortSerie[0]);  // Fermer l'extrémité de lecture 

    while(1)
    {
        write_byte = getchar();
        write(pipeEcrirePortSerie[1], &write_byte, 1);


        read(pipeLirePortSerie[0], &read_byte, 1);
        printf("%c", read_byte);

    }
}


void codeDuProcessusEnfant2(void)
{
    
    // Read data from serial port 
    int i = 0;
	tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
    char read_byte = 0;
	char  bytes_read = 0;    // Number of bytes read by the read() system call 
	

    int bytes_written = 0;
    char write_byte = 0;

    close(pipeLirePortSerie[0]);  // Fermer l'extrémité de lecture  
    close(pipeEcrirePortSerie[1]);  // Fermer l'extrémité d'écriture 

    while (1)
    {
        read(pipeEcrirePortSerie[0], &write_byte, 1);

    	bytes_written = write(fd, &write_byte, sizeof(write_byte)); // use write() to send data to port 
	    									// "fd"                   - file descriptor pointing to the opened serial port
		    								//	"write_buffer"         - address of the buffer containing data
			    							// "sizeof(write_buffer)" - No of bytes to write 


        bytes_read = read(fd, &read_byte, 1); // Read the data 
        write(pipeLirePortSerie[1], &read_byte, 1);
        read_byte  = 0x00;
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
    pid_t pid1;
    pid_t pid2;

    InitPortSerie();


    // Créer le pipe
    if (pipe(pipeLirePortSerie) == -1) {
        perror("pipe");
    }

    if(pipe(pipeEcrirePortSerie) == -1)
    {
        perror("pipe");
    }

    pid1 = fork();

    // Appel fonction Enfant
    if(pid1 == -1)
    {
        perror("fork");
    }
    else if(pid1 == 0)
    {
        codeDuProcessusEnfant();
    }
    else
    {
        pid2 = fork();

        if(pid2 == -1)
        {
            perror("fork2");
        }
        else if(pid2 == 0)
        {
            codeDuProcessusEnfant2();
        }
        else 
        {
            codeDuProcessusParent();
            wait(NULL);
            wait(NULL);
        }
    }
    

    close(fd); // Close the serial port
    printf("Fin du processus principal\n");
}




