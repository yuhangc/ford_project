/* A simple interface for connecting to arduino.

   derived from:
   http://chrisheydrick.com/2012/06/24/how-to-read-serial-data-from-an-arduino-in-linux-with-c-part-4/
*/


#include "CArduinoDevice.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <iostream>


CArduinoDevice::CArduinoDevice(const std::string& port, const BaudRate& baud) :
    PORT(port),
    BAUD(baud)
{
    buffer_size_ = 1024;
    buffer_ = (char*)malloc(buffer_size_);
    connected_ = false;
}


CArduinoDevice::~CArduinoDevice()
{
    free(buffer_);
    disconnect();
}


bool CArduinoDevice::connect()
{    
    /* open serial port */
     fd_ = ::open(PORT.c_str(), O_RDWR | O_NOCTTY);

     if (fd_ < 0) {
         printWarning("Could not connect to port"+PORT+
                      ". Check permissions. Try \"sudo chmod 666 "+PORT+"\"");
         disconnect();
         return false;
     }

     /* wait for the Arduino to reboot */
     usleep(3500000);

     /* get current serial port settings */
     struct termios toptions;
     tcgetattr(fd_, &toptions);
     /* set 9600 baud both ways */

    /* 9600 baud */
     switch (BAUD) {
     case BAUD_300:
         cfsetispeed(&toptions, B300);
         cfsetospeed(&toptions, B300);
         break;
     case BAUD_1200:
         cfsetispeed(&toptions, B1200);
         cfsetospeed(&toptions, B1200);
         break;
     case BAUD_2400:
         cfsetispeed(&toptions, B2400);
         cfsetospeed(&toptions, B2400);
         break;
     case BAUD_4800:
         cfsetispeed(&toptions, B4800);
         cfsetospeed(&toptions, B4800);
         break;
     case BAUD_9600:
         cfsetispeed(&toptions, B9600);
         cfsetospeed(&toptions, B9600);
         break;
     case BAUD_19200:
         cfsetispeed(&toptions, B19200);
         cfsetospeed(&toptions, B19200);
         break;
     case BAUD_38400:
         cfsetispeed(&toptions, B38400);
         cfsetospeed(&toptions, B38400);
         break;
     case BAUD_57600:
         cfsetispeed(&toptions, B57600);
         cfsetospeed(&toptions, B57600);
         break;
     case BAUD_115200:
         cfsetispeed(&toptions, B115200);
         cfsetospeed(&toptions, B115200);
         break;
     default:
         printWarning("unknown baud rate");
     }

      /* 8 bits, no parity, no stop bits */
      toptions.c_cflag &= ~PARENB;
      toptions.c_cflag &= ~CSTOPB;
      toptions.c_cflag &= ~CSIZE;
      toptions.c_cflag |= CS8;

      /* Canonical mode */
      toptions.c_lflag |= ICANON;

      /* commit the serial port settings */
      tcsetattr(fd_, TCSANOW, &toptions);

      connected_ = true;

      return true;
}


void CArduinoDevice::disconnect()
{
    int flag = ::close(fd_);
    /*
    if ( flag<0 ) {
        printWarning("Error closing file descriptor.");
    }
    */
    connected_ = false;
}


bool CArduinoDevice::isConnected()
{
    return connected_;
}


int CArduinoDevice::read(std::string& message)
{
    int n = read(buffer_);
    message = std::string(buffer_,n);
    return n;
}


int CArduinoDevice::read(char* message)
{
    int n = ::read(fd_, message, buffer_size_);
    if (n<=0) {
        printWarning("read error. disconnecting.");
        disconnect();
    }
    return n;
}

int CArduinoDevice::write(const std::string& message)
{
    return write(message.c_str(), message.size());
}


int CArduinoDevice::write(const char* message, int length)
{
    int n = ::write(fd_, message, length);
    if (n<=0){
        printWarning("write error. disconnecting");
        disconnect();
    }
    return n;
}


bool CArduinoDevice::setBufferSize(const int buffer_size)
{
    buffer_size_ = buffer_size;
}


void CArduinoDevice::printWarning(const std::string& message)
{
    std::cout << "WARNING. CArduinoDevice. " << message << std::endl;
}
