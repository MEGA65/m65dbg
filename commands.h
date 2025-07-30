/* vim: set expandtab shiftwidth=2 tabstop=2: */

#include <stdbool.h>

void listSearch(void);
void cmdRawHelp(void);
void cmdHelp(void);
void cmdDump(void);
void cmdMDump(void);
void cmdAssemble(void);
void cmdDisassemble(void);
void cmdMDisassemble(void);
void cmdSoftContinue(void);
void cmdContinue(void);
bool cmdGetContinueMode(void);
void cmdSetContinueMode(bool val);
void cmdStep(void);
void cmdHardNext(void);
void cmdNext(void);
void cmdFinish(void);
void cmdPrintByte(void);
void cmdPrintWord(void);
void cmdPrintDWord(void);
void cmdPrintQWord(void);
void cmdPrintString(void);
void cmdPrintBasicVar(void);
void cmdPrintMByte(void);
void cmdPrintMWord(void);
void cmdPrintMDWord(void);
void cmdPrintMQWord(void);
void cmdPrintMString(void);
void cmdPrintMFloat(void);
void cmdClearScreen(void);
void cmdAutoClearScreen(void);
void cmdSetBreakpoint(void);
void cmdSetSoftwareBreakpoint(void);
void cmdWatchByte(void);
void cmdWatchWord(void);
void cmdWatchDWord(void);
void cmdWatchQWord(void);
void cmdWatchString(void);
void cmdWatchDump(void);
void cmdWatchMByte(void);
void cmdWatchMWord(void);
void cmdWatchMDWord(void);
void cmdWatchMQWord(void);
void cmdWatchMString(void);
void cmdWatchMFloat(void);
void cmdWatchMDump(void);
void cmdWatches(void);
void cmdDeleteWatch(void);
void cmdAutoWatch(void);
void cmdSymbolValue(void);
void cmdSave(void);
void cmdLoad(void);
void cmdPoke(void);
void cmdPokeW(void);
void cmdPokeD(void);
void cmdPokeQ(void);
void cmdMPoke(void);
void cmdMPokeW(void);
void cmdMPokeD(void);
void cmdMPokeQ(void);
void cmdBackTrace(void);
void cmdUpFrame(void);
void cmdDownFrame(void);
void cmdSearch(void);
void cmdScreenshot(void);
void cmdType(void);
void cmdFtp(void);
void cmdPetscii(void);
void cmdFastMode(void);
void cmdScope(void);
void cmdOffs(void);
void cmdPrintValue(void);
void cmdForwardDis(void);
void cmdBackwardDis(void);
void cmdMCopy(void);
void cmdLocals(void);
void cmdAutoLocals(void);
void cmdMapping(void);
void cmdSeam(void);
void cmdBasicList(void);
void cmdSprite(void);
void cmdChar(void);
void cmdSet(void);
void cmdGo(void);
void cmdPalette(void);
void cmdHyppo(void);
void cmdReload(void);
void cmdRomW(void);
int doOneShotAssembly(char* strCommand);
int  cmdGetCmdCount(void);
char* cmdGetCmdName(int idx);
int isValidMnemonic(char* str);

#define BUFSIZE 65536

extern char outbuf[];
extern char inbuf[];
extern bool ctrlcflag;

// Define a struct to hold bitfield details
typedef struct {
    const char* name;             // Name of the bitfield
    int gotox_flag;               // Whether GOTOX needs to be cleared or set for this bitfield
    int byte_index1;           // First byte index
    int start_bit1;            // Start bit in the first byte
    int num_bits1;             // Number of bits in the first byte
    int byte_index2;           // Second byte index (if applicable)
    int start_bit2;            // Start bit in the second byte
    int num_bits2;             // Number of bits in the second byte
} BitfieldInfo;

typedef struct
{
  int pc;
  int a;
  int x;
  int y;
  int z;
  int b;
  int sp;
  int mapl;
  int maph;
  int lastop;
  int odd1;   // what is this?
  int odd2;   // what is this?
  char flags[16];
} reg_data;

typedef struct hdet {
  const char* name;
  int addr;
  int a;
  int y;
  bool (*inputfn)(struct hdet* service);
  void (*outputfn)(reg_data* reg);
  char* help;
} hyppo_det;


typedef struct
{
  char* name;
  void (*func)(void);
  char* params;
  char* help;
} type_command_details;

typedef struct tse
{
  char* symbol;
  int addr;   // integer value of symbol
  char* sval; // string value of symbol
  struct tse* next;
} type_symmap_entry;

typedef struct tseg
{
  char name[64];
  int offset;
} type_segment;

typedef struct t_o
{
  char modulename[256];
  type_segment segments[32];
  int seg_cnt;
  int enabled;
  struct t_o* next;
} type_offsets;

typedef struct {
  char eleven[64];
  char b65[4];
} TYP_MAP_ENTRY_11_TO_B65;


typedef enum { TYPE_BYTE, TYPE_WORD, TYPE_DWORD, TYPE_QWORD, TYPE_STRING, TYPE_DUMP,
    TYPE_MBYTE, TYPE_MWORD, TYPE_MDWORD, TYPE_MQWORD, TYPE_MSTRING, TYPE_MDUMP, TYPE_MFLOAT } type_watch;
extern char* type_names[];

typedef struct we
{
  type_watch type;
  bool show_decimal;
  bool show_char;
  bool show_binary;
  char* name;
  char* param1;
  struct we* next;
} type_watch_entry;

extern type_command_details command_details[];
extern type_symmap_entry* lstSymMap;
extern type_watch_entry* lstWatches;
