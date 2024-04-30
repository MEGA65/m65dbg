// serial code routine borrowed from:
// http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c


// Note1: enable unix domain socket support is untested on Windows/Cygwin so
// it's better to leave commented out by default ...
// -------------------------------------------------
// Note2 (GI): I'm leaving this always enabled now, as I've gotten 
// unix-sockets to work in winxp+cygwin
#define SUPPORT_UNIX_DOMAIN_SOCKET

#define _BSD_SOURCE _BSD_SOURCE
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#ifdef WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#endif
#include <unistd.h>
#include <stdio.h>
#include "serial.h"

#define error_message printf

#ifdef WINDOWS
PORT_TYPE fd = { WINPORT_TYPE_INVALID, INVALID_HANDLE_VALUE, INVALID_SOCKET };
#else
PORT_TYPE fd=-1;
#endif

bool unix_socket_flag = false;

#ifndef WINDOWS
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                error_message ("error %d from tcgetattr\n", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                error_message ("error %d from tcsetattr\n", errno);
                return -1;
        }
        return 0;
}

void set_blocking_serial (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                error_message ("error %d from tggetattr\n", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 2 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                error_message ("error %d setting term attributes\n", errno);
}
#endif

/*
	borrowed from: https://www.binarytides.com/hostname-to-ip-address-c-sockets-linux/
	Get ip from domain name
 */

int hostname_to_ip(char * hostname , char* ip)
{
	struct hostent *he;
	struct in_addr **addr_list;
	int i;
		
	if ( (he = gethostbyname( hostname ) ) == NULL) 
	{
		// get the host info
#ifndef WINDOWS
		herror("gethostbyname");
#endif
		return 1;
	}

	addr_list = (struct in_addr **) he->h_addr_list;
	
	for(i = 0; addr_list[i] != NULL; i++) 
	{
		//Return the first one;
		strcpy(ip , inet_ntoa(*addr_list[i]) );
		return 0;
	}
	
	return 1;
}

#ifdef WINDOWS
// borrowed from m65common.c

void print_error(const char* context)
{
  DWORD error_code = GetLastError();
  char buffer[256];
  DWORD size = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_MAX_WIDTH_MASK, NULL, error_code,
      MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US), buffer, sizeof(buffer), NULL);
  if (size == 0) {
    buffer[0] = 0;
  }
  fprintf(stderr, "%s: %s\n", context, buffer);
}

// Opens the specified serial port, configures its timeouts, and sets its
// baud rate.  Returns a handle on success, or INVALID_HANDLE_VALUE on failure.
HANDLE open_serial_port(const char* device, uint32_t baud_rate)
{
  // COM10+ need to have \\.\ added to the front
  // (see
  // https://support.microsoft.com/en-us/topic/howto-specify-serial-ports-larger-than-com9-db9078a5-b7b6-bf00-240f-f749ebfd913e
  // and https://github.com/MEGA65/mega65-tools/issues/48)
  char device_with_prefix[8192];
  snprintf(device_with_prefix, 8192, "\\\\.\\%s", device);

  HANDLE port = CreateFileA(
      device_with_prefix, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (port == INVALID_HANDLE_VALUE) {
    print_error(device);
    return INVALID_HANDLE_VALUE;
  }

  // Consider making the serial buffers bigger? They default to 8192?
  // - https://stackoverflow.com/questions/54313240/why-is-my-serial-read-on-windows-com-port-limited-to-8192-bytes
  // That SO thread suggests making use of SetupComm() to specify the buffer-size you want:
  // - https://docs.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-setupcomm

  SetupComm(port, 131072, 131072);

  // Flush away any bytes previously read or written.
  BOOL success = FlushFileBuffers(port);
  if (!success) {
    print_error("Failed to flush serial port");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  // Configure read and write operations to time out after 1 ms and 100 ms, respectively.
  COMMTIMEOUTS timeouts = { 0 };
  timeouts.ReadIntervalTimeout = 0;
  timeouts.ReadTotalTimeoutConstant = 1;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 100;
  timeouts.WriteTotalTimeoutMultiplier = 0;

  success = SetCommTimeouts(port, &timeouts);
  if (!success) {
    print_error("Failed to set serial timeouts");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  DCB state;
  state.DCBlength = sizeof(DCB);
  success = GetCommState(port, &state);
  if (!success) {
    print_error("Failed to get serial settings");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  state.fBinary = TRUE;
  state.fDtrControl = DTR_CONTROL_ENABLE;
  state.fDsrSensitivity = FALSE;
  state.fTXContinueOnXoff = FALSE;
  state.fOutX = FALSE;
  state.fInX = FALSE;
  state.fErrorChar = FALSE;
  state.fNull = FALSE;
  state.fRtsControl = RTS_CONTROL_ENABLE;
  state.fAbortOnError = FALSE;
  state.fOutxCtsFlow = FALSE;
  state.fOutxDsrFlow = FALSE;
  state.ByteSize = 8;
  state.StopBits = ONESTOPBIT;
  state.Parity = NOPARITY;

  state.BaudRate = baud_rate;

  success = SetCommState(port, &state);
  if (!success) {
    print_error("Failed to set serial settings");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  return port;
}
#endif

/**
 * opens the desired serial port at the required 2000000 bps, or to a unix-domain socket
 *
 * portname = the desired "/dev/ttyS*" device portname to use
 *            "unix#..path.." defines a unix-domain named stream socket to connect to (emulator)
 */
bool serialOpen(char* portname)
{
#ifdef WINDOWS
  fd=open_the_serial_port(portname,2000000);
  if (fd==INVALID_HANDLE_VALUE) {
    fprintf(stderr,"Could not open serial port '%s'\n",portname);
    exit(-1);
  }
#else
  if (!strncasecmp(portname, "tcp", 3))
  {
    char hostname[128] = "localhost";
    int port = 4510;  // assume a default port of 4510
    if (portname[3] == '#') // did user provide a hostname and port number?
    {
      sscanf(&portname[4], "%[^:]:%d", hostname, &port);
    }
    else if (portname[3] == '\\' && portname[4] == '#')
    {
      sscanf(&portname[5], "%[^:]:%d", hostname, &port);
    }

    struct sockaddr_in sock_st;
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      error_message("error %d creating tcp/ip socket: %s\n", errno, strerror (errno));
      return false;
    }

		char ip[100];
		
		hostname_to_ip(hostname , ip);
		printf("%s resolved to %s" , hostname , ip);

    sock_st.sin_addr.s_addr = inet_addr(ip);
    sock_st.sin_family = AF_INET;
    sock_st.sin_port = htons(port);

    if (connect(fd, (struct sockaddr*)&sock_st, sizeof(sock_st)) < 0)
    {
      error_message("error %d connecting to tcp/ip socket %s:%d: %s\n", errno, hostname, port, strerror (errno));
      close(fd);
      return false;
    }
  }
  else if (!strncasecmp(portname, "unix#", 5) ||
      !strncasecmp(portname, "unix\\#", 6)) {
#ifdef SUPPORT_UNIX_DOMAIN_SOCKET
    struct sockaddr_un sock_st;
    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
      error_message("error %d creating UNIX-domain socket: %s\n", errno, strerror (errno));
      return false;
    }
    unix_socket_flag = true;
    sock_st.sun_family = AF_UNIX;
    int hashloc = strchr(portname, '#') - portname;
    strcpy(sock_st.sun_path, portname + hashloc + 1);
    if (connect(fd, (struct sockaddr*)&sock_st, sizeof(struct sockaddr_un))) {
      error_message("error %d connecting to UNIX-domain socket %s: %s\n", errno, portname + 5, strerror (errno));
      close(fd);
      return false;
    }
    //set_blocking_std (fd, 0);    // set no blocking
#else
    error_message("unix domain socket is not compiled in this time!\n");
    return false;
#endif
  } else {
    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
          error_message ("error %d opening %s: %s\n", errno, portname, strerror (errno));
          return false;
    }
#ifdef __APPLE__
    // WARNING: This slower bps won't work with newer bitstreams
    set_interface_attribs (fd, B230400, 0);  // set speed to slower 230,400 bps, 8n1 (no parity)
#else
    set_interface_attribs (fd, B2000000, 0);  // set speed to 2,000,000 bps, 8n1 (no parity)
#endif
    set_blocking_serial (fd, 0);	// set no blocking
    // borrowed this line from m65.c - set_serial_speed(), as it seems to set the
    // non-blocking behaviour properly
    //fcntl(fd,F_SETFL,fcntl(fd, F_GETFL, NULL)|O_NONBLOCK);
  }
#endif 
  return true;
}

#ifndef WINDOWS
void serialBaud(bool fastmode)
{
#ifndef __CYGWIN__
  if (fastmode)
    set_interface_attribs(fd, B4000000, 0);
  else
    set_interface_attribs(fd, B2000000, 0);
#endif
}
#endif


/**
 * closes the opened serial port
 */
bool serialClose(void)
{
  if (fd >= 0)
  {
#ifdef WINDOWS
    CloseHandle(fd);
#else
    close(fd);
#endif
    fd = 0;
    return true;
  }

  return false;
}

void serialFlush(void)
{
  // http://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
  //  sleep(2); //required to make flush work, for some reason (for USB serial ports?)
//  tcflush(fd,TCIOFLUSH);

#ifdef WINDOWS
  FlushFileBuffers(fd);
#else
  // I'll now try a 'manual' flush, to see if that works for Ralph's mac and my mac...
  int bytes_available = 0;
  static char tmp[16384];
#ifdef FIONREAD
  ioctl(fd, FIONREAD, &bytes_available);
#else
  ioctl(fd, TIOCINQ, &bytes_available);
#endif
  if (bytes_available > 0)
    read(fd, tmp, bytes_available);
#endif
}

#ifdef WINDOWS
int win_serial_port_write(HANDLE port, uint8_t* buffer, size_t size, const char* func, const char* file, const int line)
{
  DWORD offset = 0;
  DWORD written;
  BOOL success;
  //  printf("Calling WriteFile(%d)\n",size);

  // if (debug_serial) {
  //   fprintf(stderr, "%s:%d:%s(): ", file, line, func);
  //   dump_bytes(0, "serial write (windows)", buffer, size);
  // }

  while (offset < size) {
    success = WriteFile(port, &buffer[offset], size - offset, &written, NULL);
    //  printf("  WriteFile() returned.\n");
    if (!success) {
      print_error("Failed to write to port");
      return -1;
    }
    if (written > 0)
      offset += written;
    if (offset < size) {
      // Assume buffer is full, so wait a little while
      //      usleep(1000);
    }
  }
  success = FlushFileBuffers(port);
  if (!success)
    print_error("Failed to flush buffers");
  return size;
}

int win_tcp_write(SOCKET sock, uint8_t* buffer, size_t size, const char* func, const char* file, const int line)
{
  // if (debug_serial) {
  //   fprintf(stderr, "%s:%d:%s(): ", file, line, func);
  //   dump_bytes(0, "tcp write (windows)", buffer, size);
  // }

  int iResult = send(sock, (char*)buffer, size, 0);
  if (iResult == SOCKET_ERROR) {
    printf("send failed with error: %d\n", WSAGetLastError());
    closesocket(sock);
    WSACleanup();
    exit(1);
  }
  int count = iResult;
  return count;
}
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int do_serial_port_write(WINPORT port, uint8_t* buffer, size_t size)
{
  if (port.type == WINPORT_TYPE_FILE)
    return win_serial_port_write(port.fdfile, buffer, size);
  else if (port.type == WINPORT_TYPE_SOCK)
    return win_tcp_write(port.fdsock, buffer, size);
  return 0;
}
#endif

/**
 * writes a string to the serial port
 */
void serialWrite(char* string)
{ 
  serialFlush();

  int i = strlen(string);
  // do we need to add a carriage return to the end?
  if (string[i-1] != '\n')
  {
    strcat(string, "\n");
    i++;
  }

#ifdef WINDOWS
  do_serial_port_write(fd, string, i);
#else
  write (fd, string, i);           // send string
#endif
}


/**
 * reads serial data and feeds it into the provided buffer. The routine will read up
 * until the next '.' prompt. It should also crop out the first line, which is just
 * and echo of the command.
 *
 * returns:
 *   true = read till the next '.' prompt.
 *   false = could not read till next '.' prompt (eg, buffer was filled)
 */
bool serialRead(char* buf, int bufsize)
{
  char* ptr = buf;
  char* secondline = NULL;
  bool foundLF = false;

  while (ptr - buf < bufsize)
  {
    int n = read (fd, ptr, bufsize);  // read up to 'bufsize' characters if ready to read

    if (n == -1)
      return false;

    // check for "." prompt
    for (int k = 0; k < n; k++)
    {
      if ( *(ptr+k) == '\n' )
      {
        foundLF = true;
        if (!secondline)
          secondline = ptr+k+1;
      }
      else if (foundLF && *(ptr+k) == '.')
      {
        *(ptr+k) = '\0';

        int len = strlen(secondline) + 1;
        for (int z = 0; z < len; z++)
          *(buf+z) = *(secondline+z);
        return true;
      }
      else
        foundLF = false;
    }

    ptr += n;
  }

  return false;
}
