/*!
* \file rmp_io.h
* \author  William Woodall <wjwwood@gmail.com>
* \version 1.0
*
* \section LICENSE
*
* The BSD License
*
* Copyright (c) 2013 William Woodall
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the 
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
* sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
* DEALINGS IN THE SOFTWARE.
*
* \section DESCRIPTION
*
* This provides an abstract I/O interface for communicating with the RMP.
* 
* This library depends on Boost: http://www.boost.org/
* and possibly depends on a Serial library: https://github.com/wjwwood/serial
* depending on the library build configuration.
*/

#ifndef RMP_IO_H
#define RMP_IO_H

#include <vector>
#include <cstdio>

#include <boost/thread.hpp>

#include <segwayrmp/segwayrmp.h>

namespace segwayrmp {

/*!
* Represents the structure of a usb packet to be sent or received over
* the wire.
*/
struct Packet {
  unsigned short id; /*!< Packet ID. */
  unsigned char channel; /*!< CAN Bus Channel. */
  unsigned char data[8]; /*!< Data bytes. */

  Packet() : id(0), channel(0) {
    for (int i = 0; i < 8; ++i) { data[i] = 0x00; }
  }
};

/*!
* Provides a generic interface for getting, building, manipulating, and sending packets.
*/
class RMPIO {
public:
    RMPIO() : canceled(false) {}
  /*!
   * Abstract Connect Function, implemented by subclass.
   */
  virtual void connect() = 0;
  
  /*!
   * Abstract Disconnect Function, implemented by subclass.
   */
  virtual void disconnect() = 0;
  
  /*!
   * Abstract Read Function, implemented by subclass.
   * 
   * \param buffer An unsigned char array for data to be read into.
   * \param size The amount of data to be read.
   * \return int Bytes read.
   */
  virtual int read(unsigned char* buffer, int size) = 0;
  
  /*!
   * Abstract Write Function, implemented by subclass.
   * 
   * \param buffer An unsigned char array of data to be written.
   * \param size The amount of data to be written.
   * \return int Bytes written.
   */
  virtual int write(unsigned char* buffer, int size) = 0;
  
  /*!
   * This function reads from the RMP and returns one complete packet.
   * 
   * \param packet A packet by reference to be read into.
   */
  void getPacket(Packet &packet);
  
  /*!
   * This function validates and writes a packet to the RMP.
   * 
   * \param packet A packet by reference to be written.
   */
  void sendPacket(Packet &packet);
  
  /*!
   * A function to see if the underlying I/O interface is connected.
   * 
   * \return bool weather or not the underlying I/O interface is connected.
   */
  bool isConnected() {return this->connected;}

  /*!
   * Cancels any currently being processed packets, should be called at shudown.
   */
  void cancel() {this->canceled = true;}
  
protected:
  void fillBuffer();
  unsigned char computeChecksum(unsigned char* usb_packet);
  
  bool connected;
  bool canceled;
  
  std::vector<unsigned char> data_buffer;
};

DEFINE_EXCEPTION(PacketRetrievalException, "Error retrieving a packet from the"
  " SegwayRMP: ", "Unspecified");

} // Namespace segwayrmp

#endif


#ifndef RMP_SERIAL_H
#define RMP_SERIAL_H

#include "rmp_io.h"

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

namespace segwayrmp {

/*!
 * Provides a serial based interface for reading and writing packets.
 */
class SerialRMPIO : public RMPIO {
public:
    /*!
     * Constructs the SerialRMPIO object.
     */
    SerialRMPIO();
    ~SerialRMPIO();
    
    /*!
     * Connects to the serial port if it has been configured. Can throw ConnectionFailedException.
     */
    void connect();
    
    /*!
     * Disconnects from the serial port if it is open.
     */
    void disconnect();
    
    /*!
     * Read Function, reads from the serial port.
     * 
     * \param buffer An unsigned char array for data to be read into.
     * \param size The amount of data to be read.
     * \return int Bytes read.
     */
    int read(unsigned char* buffer, int size);
    
    /*!
     * Write Function, writes to the serial port.
     * 
     * \param buffer An unsigned char array of data to be written.
     * \param size The amount of data to be written.
     * \return int Bytes written.
     */
    int write(unsigned char* buffer, int size);
    
    /*!
     * Configures the serial port.
     * 
     * \param port The com port identifier like '/dev/ttyUSB0' on POSIX and like 'COM1' on windows.
     * \param baudrate The speed of the serial communication.
     */
    void configure(std::string port, int baudrate);
    
private:
    bool configured;
    
    std::string port;
    int baudrate;

    bool connected = false;
    struct termios tty;
    
    int serial_port;
};

}

#endif