/**
 * serial.h
 */

#include <stdbool.h>

#ifdef WINDOWS
#define WINPORT_TYPE_INVALID -1
#define WINPORT_TYPE_FILE 0
#define WINPORT_TYPE_SOCK 1
typedef struct {
  int type; // 0 = file, 1 = socket
  HANDLE fdfile;
  SOCKET fdsock;
} WINPORT;

#define PORT_TYPE WINPORT

#else
#define PORT_TYPE int
#endif

bool serialOpen(char* portName);
bool serialClose(void);
void serialWrite(char* string);
bool serialRead(char* buf, int bufsize);
void serialBaud(bool fastmode);
