#ifndef CARDUINODEVICE_H_
#define CARDUINODEVICE_H_

#include <string>

class CArduinoDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    enum BaudRate {
        BAUD_300,
        BAUD_1200,
        BAUD_2400,
        BAUD_4800,
        BAUD_9600,
        BAUD_19200,
        BAUD_38400,
        BAUD_57600,
        BAUD_115200
    };

    //! Constructor of cArduinoDevice.
    CArduinoDevice(const std::string& port = "/dev/ttyACM0", const BaudRate& baud = BAUD_9600);

    //! Destructor of cArduinoDevice.
    virtual ~CArduinoDevice();

    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Open connection to device.
    bool connect();

    //! Close connection to device.
    void disconnect();

    //! Get last values of device.
    bool isConnected();

    //! Read serial port. returns number of bytes read
    int read(std::string& message);
    int read(char* message);

    //! Write to serial port. returns number of bytes written
    int write(const std::string& message);
    int write(const char* message, int length);

    //! Set maximum number or read bytes
    bool setBufferSize(const int buffer_size);

protected:

    void printWarning(const std::string& message);

    const std::string PORT;
    const BaudRate BAUD;

    //! read buffer
    char* buffer_;
    int buffer_size_;

    //! file descriptor
    int fd_;

    bool connected_;
};

#endif
