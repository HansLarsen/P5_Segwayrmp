#include "segwayrmp/segwayrmp.h"
#include "segwayrmp/impl/rmp_serial.h"

using namespace segwayrmp;

/////////////////////////////////////////////////////////////////////////////
// SerialRMPIO

SerialRMPIO::SerialRMPIO() : configured(false), baudrate(460800), port("") {
  this->connected = false;
}

SerialRMPIO::~SerialRMPIO() {
  this->disconnect();
}

void SerialRMPIO::configure(std::string port, int baudrate) {
  this->port = port;
  this->baudrate = baudrate;
  this->configured = true;
}

void SerialRMPIO::connect() {


  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  try {
      serial_port = open(port.c_str(), O_RDWR);

      if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
      }

      if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      }

      cfsetispeed(&tty, B460800);
      printf("Baudrate not imp set to B460800");


      // Configure and open the serial port
      //serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      //this->serial_port.setTimeout(timeout);
      //this->serial_port.open();

  } catch(std::exception &e) {
      RMP_THROW_MSG(ConnectionFailedException, e.what());
  }

  this->connected = true;
}

void SerialRMPIO::disconnect() {
  if(this->connected) {
      close(serial_port);
      //if(this->serial_port.isOpen())
      //    this->serial_port.close();
      this->connected = false;
  }
}

int readwrapper(int serial_port, unsigned char* buffer, int size) {
  return read(serial_port, buffer, size);
}

int SerialRMPIO::read(unsigned char* buffer, int size) {
  //return this->serial_port.read(buffer, size);
  return readwrapper(serial_port, buffer, size);
}

int writewrapper(int serial_port, unsigned char* buffer, int size) {
  return write(serial_port, buffer, size);
}

int SerialRMPIO::write(unsigned char* buffer, int size) {
  //return this->serial_port.write(buffer, size);
  return writewrapper(serial_port, buffer, size);
}
