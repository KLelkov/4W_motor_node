#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>


class CppSerial
{

private:
  int tty_fid = -1;  // tty FIle Descriptor
  unsigned char ch = '0';  // single char read\write holder
  unsigned char read_buffer[256];  // read buffer
  unsigned char write_buffer[256];  // write buffer

  int set_interface_attribs(int speed)
  {
      struct termios tty;

      if (tcgetattr(tty_fid, &tty) < 0) {
          printf("Error from tcgetattr: %s\n", strerror(errno));
          return -1;
      }

      cfsetospeed(&tty, (speed_t)speed);
      cfsetispeed(&tty, (speed_t)speed);

      tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
      tty.c_cflag &= ~CSIZE;
      tty.c_cflag |= CS8;         /* 8-bit characters */
      tty.c_cflag &= ~PARENB;     /* no parity bit */
      tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
      tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

      /* setup for non-canonical mode */
      tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
      tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
      tty.c_oflag &= ~OPOST;

      /* fetch bytes as they become available */
      /* Block until either VMIN characters have been received,
      or VTIME after first character has elapsed. Note that the
      timeout for VTIME does not begin until the first character
      is received. */
      tty.c_cc[VMIN] = 10;
      tty.c_cc[VTIME] = 10;

      if (tcsetattr(tty_fid, TCSANOW, &tty) != 0) {
          printf("Error from tcsetattr: %s\n", strerror(errno));
          return -1;
      }
      return 0;
  }  // int set_interface_attribs(int speed)

  void set_mincount(int mcount)
  {
      struct termios tty;

      if (tcgetattr(tty_fid, &tty) < 0) {
          printf("Error tcgetattr: %s\n", strerror(errno));
          return;
      }

      tty.c_cc[VMIN] = mcount ? 1 : 0;
      tty.c_cc[VTIME] = 5;        /* half second timer */

      if (tcsetattr(tty_fid, TCSANOW, &tty) < 0)
          printf("Error tcsetattr: %s\n", strerror(errno));
  }  // set_mincount(int fd, int mcount)


public:
  // Constructor
  CppSerial()
  {
  }

  void Open(char portname[], int baud_rate) {
      tty_fid = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK | O_SYNC);
      int k = 0;
      while (tty_fid < 0) {
          printf("Error opening %s: %s\n", portname, strerror(errno));
          usleep (1000000);   // micro seconds
          tty_fid = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK | O_SYNC);
          k++;
          if (k > 4) {
              exit(EXIT_FAILURE);
          }
      }
      set_interface_attribs(baud_rate);
  }

  ~CppSerial()
  {
    close(tty_fid);
  }

  int ReadLine(unsigned char* outstring)
  {
    if (tty_fid == -1)
    {
      printf("Please use .Open() to open the UART before attempting to read data\n");
      return -1;
    }
    int index = 0;
    while (read(tty_fid, &ch, 1) > 0)
    {
      printf("%c", ch);// ch << std::endl;
      if (ch == '\r')
      {
        break;
      }
      else
      {
        outstring[index++] = ch;
      }
    }
    if (index == 0)
      return -1;
    else
      return index;
  }

  int Read(unsigned char* outstring)
  {
    if (tty_fid == -1)
    {
      printf("Please use .Open() to open the UART before attempting to read data\n");
      return -1;
    }
    int bytes_available;
    ioctl(tty_fid, FIONREAD, &bytes_available);
    int n = read(tty_fid, read_buffer, bytes_available);
    for (int i = 0; i < n; i++)
    {
      //printf("%c", read_buffer[i]);
      outstring[i] = read_buffer[i];
    }
    return n;
  }

  void Send(unsigned char* buffer, int size) {
    if (tty_fid == -1)
    {
      printf("Please use .Open() to open the UART before attempting to send data\n");
      return;
    }
    write(tty_fid, buffer, size);
    return;
  }

  void SendBinary() {
    if (tty_fid == -1)
    {
      printf("Please use .Open() to open the UART before attempting to send data\n");
      return;
    }
    // TODO:
    return;
  }

  int ReadBinary() {
    if (tty_fid == -1)
    {
      printf("Please use .Open() to open the UART before attempting to read data\n");
      return -1;
    }
    // TODO:
    return 0;
  }

  void FlushReceive()
  {
    // 0 - flush receive
    // 1 - flush transmit
    // 2 - flush both
    ioctl(tty_fid, TCFLSH, 0); // flush receive
    return;
  }

  void FlushTransmit()
  {
    // 0 - flush receive
    // 1 - flush transmit
    // 2 - flush both
    ioctl(tty_fid, TCFLSH, 1); // flush transmit
    return;
  }

};
