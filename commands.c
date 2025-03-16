/* vim: set expandtab shiftwidth=2 tabstop=2: */

#define _BSD_SOURCE _BSD_SOURCE
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>
#include <math.h>
#include "commands.h"
#include "serial.h"
#include "gs4510.h"
#include "screen_shot.h"
#include "m65.h"

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KINV  "\x1B[7m"
#define KINV_OFF "\x1B[27m"
#define KCLEAR "\x1B[2J"
#define KPOS0_0 "\x1B[1;1H"


int get_sym_value(char* token);
void print_char(int c);
int parseBinaryString(char* str);
unsigned char* get_palette(void);


typedef struct
{
  int addr;
  unsigned int b[16];
} mem_data;

typedef struct
{
  int addr;
  char name[256];
} rom_chunk;

typedef struct {
  bool found;
  int start_bit;
  int num_bits;
} poke_bitfield_info;

rom_chunk rom_chunks[] = {
  { 0x20000, "(C65 DOS chunk 1)" },
  { 0x22000, "(C65 DOS chunk 2)" },
  { 0x24000, "(C65 BASIC ALT chunk 1)" },
  { 0x26000, "(C65 BASIC ALT chunk 2)" },
  { 0x28000, "(CHARSET-A)" },
  { 0x2a000, "(C64 BASIC)" },
  { 0x2c000, "(C65 DOS chunk 3 + C65 KERNAL chunk 1 + CHARSET-C)" },
  { 0x2e000, "(C64 KERNAL)" },
  { 0x30000, "(C65 MONITOR)" },
  { 0x32000, "(C65 BASIC chunk 1)" },
  { 0x34000, "(C65 BASIC chunk 2)" },
  { 0x36000, "(C65 BASIC chunk 3)" },
  { 0x38000, "(C65 BASIC chunk 4)" },
  { 0x3a000, "(C65 BASIC chunk 5)" },
  { 0x3c000, "(C65 KERNAL chunk 2 + CHARSET-B)" },
  { 0x3e000, "(C65 EDITOR + C65 KERNAL chunk 3)" },
  { 0, "" }
};

typedef struct {
  int code;
  const char* desc;
} hyppo_err;

hyppo_err hyppo_errors[] = {
  { 0x00, "no error" },
  { 0x01, "partition not interesting" },
  { 0x02, "bad signature" },
  { 0x03, "is small FAT" },
  { 0x04, "too many reserved clusters" },
  { 0x05, "not two FATs" },
  { 0x06, "too few clusters" },
  { 0x07, "read timeout" },
  { 0x08, "partition error" },
  { 0x10, "invalid address" },
  { 0x11, "illegal value" },
  { 0x20, "read error" },
  { 0x21, "write error" },
  { 0x80, "no such drive" },
  { 0x81, "name too long" },
  { 0x82, "hyppo service not implemented" },
  { 0x83, "file too long (>16MB)" },
  { 0x84, "too many open files" },
  { 0x85, "invalid cluster" },
  { 0x86, "is a directory" },
  { 0x87, "not a directory" },
  { 0x88, "file not found" },
  { 0x89, "invalid file descriptor" },
  { 0x8a, "image wrong length" },
  { 0x8b, "image fragmented" },
  { 0x8c, "no space" },
  { 0x8d, "file exists" },
  { 0x8e, "directory full" },
  { 0x8f, "eof / no such trap" },
  { -1,   NULL }
};

// a few function prototypes
mem_data get_mem(int addr, bool useAddr28);
void print_byte_at_addr(char* token, int addr, bool useAddr28, bool show_decimal, bool show_char, bool show_binary);
void print_word_at_address(char* token, int addr, bool useAddr28, bool show_decimal);
void print_dword_at_address(char* token, int addr, bool useAddr28, bool show_decimal);
void print_qword_at_address(char* token, int addr, bool useAddr28, bool show_decimal);
char* toBinaryString(int val, poke_bitfield_info* bfi);
mem_data* get_mem28array(int addr);
int peek(unsigned int address);
void pokew(unsigned int address, int val);
void poke(unsigned int address, int val);
void set_mem(int addr, mem_data mem);


void out_errorcode(reg_data* reg)
{
  int k = 0;
  while (hyppo_errors[k].code != -1) {
    if (hyppo_errors[k].code == reg->a) {
      printf("Error code: $%02X - %s\n", reg->a, hyppo_errors[k].desc);
      return;
    }
    k++;
  }

  printf("??? Unknown error code: $%02X\n", reg->a);
}

mem_data memSetMapping;

void out_getversion(reg_data* reg)
{
  printf("Hyppo V%d.%d\n", reg->a, reg->x);
  printf("HDOS  V%d.%d\n", reg->y, reg->z);
}

bool in_set_mapping(hyppo_det *service)
{
  int mapl, maph, mblo, mbhi;
  char* token = strtok(NULL, " ");

  if (!token) {
    printf("ERROR: Missing arg 1: MAPL\n");
    printf("  - %s\n", service->help);
    return false;
  }
  mapl = get_sym_value(token);

  token = strtok(NULL, " ");
  if (!token) {
    printf("ERROR: Missing arg 2: MAPH\n");
    printf("  - %s\n", service->help);
    return false;
  }
  maph = get_sym_value(token);

  token = strtok(NULL, " ");
  if (!token) {
    printf("ERROR: Missing arg 3: MBLO\n");
    printf("  - %s\n", service->help);
    return false;
  }
  mblo = get_sym_value(token);

  token = strtok(NULL, " ");
  if (!token) {
    printf("ERROR: Missing arg 4: MBHI\n");
    printf("  - %s\n", service->help);
    return false;
  }
  mbhi = get_sym_value(token);

  // preserve original memory
  memSetMapping = get_mem(0x200, false);

  poke(0x200, (mapl >> 8) & 0xff); // NOTE: MAPL is in big-endian form
  poke(0x201, mapl & 0xff);

  poke(0x202, (maph >> 8) & 0xff); // NOTE: MAPH is in big-endian form
  poke(0x203, maph & 0xff);

  poke( 0x204, mblo);
  poke( 0x205, mbhi);

  return true;
}

void out_set_mapping(reg_data* reg)
{
  // restore original bytes
  set_mem(0x0200, memSetMapping);
}

bool in_get_mapping(hyppo_det* service)
{
  // preserve original memory
  memSetMapping = get_mem(0x200, false);
  return true;
}

void out_get_mapping(reg_data* reg)
{
  int mapl, maph, mblo, mbhi;

  mapl = (peek(0x200) << 8) | peek(0x201);
  maph = (peek(0x202) << 8) | peek(0x203);
  mblo = peek(0x204);
  mbhi = peek(0x205);

  printf("MAPH=%04X, MAPL=%04X\n", maph, mapl);
  printf("MBHI=%02X,   MBLO=%02X\n", mbhi, mblo);

  // restore original bytes
  set_mem(0x0200, memSetMapping);
}

hyppo_det hyppo_services[] = {
//  service name            addr    areg  yreg  inputfn         outputfn          help
  { "geterrorcode",         0xd640, 0x38, -1,   NULL,           out_errorcode,    NULL },
  { "getversion",           0xd640, 0x00, -1,   NULL,           out_getversion,   NULL },
  { "setup_transfer_area",  0xd640, 0x3a, -1,   NULL,           NULL,             NULL },
  { "set_mapping",          0xd640, 0x76, 0x02, in_set_mapping, out_set_mapping,  "args: <MAPL> <MAPH> <MBLO> <MBHI>" },
  { "get_mapping",          0xd640, 0x74, 0x02, in_get_mapping, out_get_mapping,  NULL },
  { NULL,                   0,      0,    -1,   NULL,           NULL,             NULL }
};

bool outputFlag = true;
bool continue_mode = false;

char outbuf[BUFSIZE] = { 0 };  // the buffer of what command is output to the remote monitor
char inbuf[BUFSIZE] = { 0 }; // the buffer of what is read in from the remote monitor

char* type_names[] = { "BYTE   ", "WORD   ", "DWORD  ", "QWORD  ", "STRING ", "DUMP   ",
                       "MBYTE  ", "MWORD  ", "MDWORD ", "MQWORD ", "MSTRING", "MDUMP  ", "MFLOAT " };

bool autocls = false; // auto-clearscreen flag
bool autowatch = false; // auto-watch flag
bool autolocals = false; // auto-locals flag
bool romw = false; // rom writable flag
bool petscii = false; // show chars in dumps based on petscii
bool fastmode = false; // switch for slow (2,000,000bps) and fast (4,000,000bps) modes
bool ctrlcflag = false; // a flag to keep track of whether ctrl-c was caught
int  traceframe = 0;  // tracks which frame within the backtrace

int dis_offs = 0;
int dis_scope = 10;

int softbrkaddr = 0;
unsigned char softbrkmem[3] = { 0 };

type_command_details command_details[] =
{
  { "?", cmdRawHelp, NULL, "Shows help information for raw/native monitor commands" },
  { "help", cmdHelp, "[<cmdname>]",  "Shows help information for m65dbg commands. If optional <cmdname> given, show help for that command only." },
  { "dump", cmdDump, "<addr16> [<count>]", "Dumps memory (CPU context) at given address (with character representation in right-column)" },
  { "mdump", cmdMDump, "<addr28> [<count>]", "Dumps memory (28-bit addresses) at given address (with character representation in right-column)" },
  { "a", cmdAssemble, "<addr28>", "Assembles instructions at the given <addr28> location." },
  { "dis", cmdDisassemble, "[<addr16> [<count>]]", "Disassembles the instruction at <addr> or at PC. If <count> exists, it will dissembly that many instructions onwards" },
  { "mdis", cmdMDisassemble, "[<addr28> [<count>]]", "Disassembles the instruction at <addr> or at PC. If <count> exists, it will dissembly that many instructions onwards" },
  { "c", cmdContinue, "[<addr>]", "continue (until optional <addr>) (equivalent to t0, but more m65dbg-friendly)"},
  { "sc", cmdSoftContinue, "[<addr>]", "soft continue (until optional <addr>) (equivalent to t0, but more m65dbg-friendly)"},
  { "step", cmdStep, "[<count>]", "Step into next instruction. If <count> is specified, perform that many steps" }, // equate to pressing 'enter' in raw monitor
  { "n", cmdNext, "[<count>]", "Step over to next instruction (software-based, slow). If <count> is specified, perform that many steps" },
  { "next", cmdHardNext, "[<count>]", "Step over to next instruction (hardware-based, fast, xemu-only, for now). If <count> is specified, perform that many steps" },
  { "finish", cmdFinish, NULL, "Continue running until function returns (ie, step-out-from)" },
  { "pb", cmdPrintByte, "<addr>", "Prints the byte-value of the given address" },
  { "pw", cmdPrintWord, "<addr>", "Prints the word-value of the given address" },
  { "pd", cmdPrintDWord, "<addr>", "Prints the dword-value of the given address" },
  { "pq", cmdPrintQWord, "<addr>", "Prints the qword-value of the given address" },
  { "ps", cmdPrintString, "<addr>", "Prints the null-terminated string-value found at the given address" },
  { "pbas", cmdPrintBasicVar, "[<varname>]", "Print the value of specified basic var. If none given, print value of all vars." },
  { "pmb", cmdPrintMByte, "<addr28>", "Prints the byte-value of the given 28-bit address" },
  { "pmw", cmdPrintMWord, "<addr28>", "Prints the word-value of the given 28-bit address" },
  { "pmd", cmdPrintMDWord, "<addr28>", "Prints the dword-value of the given 28-bit address" },
  { "pmq", cmdPrintMQWord, "<addr28>", "Prints the qword-value of the given 28-bit address" },
  { "pms", cmdPrintMString, "<addr28>", "Prints the null-terminated string-value found at the given 28-bit address" },
  { "pmf", cmdPrintMFloat, "<addr28>", "Prints the BASIC float value at the given 28-bit address" },
  { "cls", cmdClearScreen, NULL, "Clears the screen" },
  { "autocls", cmdAutoClearScreen, "0/1", "If set to 1, clears the screen prior to every step/next command" },
  { "romw", cmdRomW, "0/1", "If set to 1, rom is writable. If set to 0, rom is read-only. If no parameter, it toggles" },
  { "break", cmdSetBreakpoint, "<addr>", "Sets the hardware breakpoint to the desired address" },
  { "sbreak", cmdSetSoftwareBreakpoint, "<addr>", "Sets the software breakpoint to the desired address" },
  { "wb", cmdWatchByte, "<addr>", "Watches the byte-value of the given address" },
  { "ww", cmdWatchWord, "<addr>", "Watches the word-value of the given address" },
  { "wd", cmdWatchDWord, "<addr>", "Watches the dword-value of the given address" },
  { "wq", cmdWatchQWord, "<addr>", "Watches the qword-value of the given address" },
  { "ws", cmdWatchString, "<addr>", "Watches the null-terminated string-value found at the given address" },
  { "wdump", cmdWatchDump, "<addr> [<count>]", "Watches a dump of bytes at the given address" },
  { "wmb", cmdWatchMByte, "<addr28>", "Watches the byte-value of the given 28-bit address" },
  { "wmw", cmdWatchMWord, "<addr28>", "Watches the word-value of the given 28-bit address" },
  { "wmd", cmdWatchMDWord, "<addr28>", "Watches the dword-value of the given 28-bit address" },
  { "wmq", cmdWatchMQWord, "<addr28>", "Watches the qword-value of the given 28-bit address" },
  { "wms", cmdWatchMString, "<addr28>", "Watches the null-terminated string-value found at the given 28-bit address" },
  { "wmf", cmdWatchMFloat, "<addr28>", "Watches a BASIC float value at the given 28-bit address" },
  { "wmdump", cmdWatchMDump, "<addr28> [<count>]", "Watches an mdump of bytes at the given 28-bit address" },
  { "watches", cmdWatches, NULL, "Lists all watches and their present values" },
  { "wdel", cmdDeleteWatch, "<watch#>/all", "Deletes the watch number specified (use 'watches' command to get a list of existing watch numbers)" },
  { "autowatch", cmdAutoWatch, "0/1", "If set to 1, shows all watches prior to every step/next/dis command" },
  { "symbol", cmdSymbolValue, "<symbol|$hex>", "retrieves the value of the symbol from the .map file. Alternatively, can find symbol name/s matching given $hex address. If two $hex values are given, it finds all symbols within this range" },
  { "save", cmdSave, "<binfile> <addr28> <count>", "saves out a memory dump to <binfile> starting from <addr28> and for <count> bytes" },
  { "load", cmdLoad, "<binfile> <addr28>", "loads in <binfile> to <addr28>" },
  { "poke", cmdPoke, "<addr16> <byte/s>", "pokes byte value/s into <addr16> (and beyond, if multiple values)" },
  { "pokew", cmdPokeW, "<addr16> <word/s>", "pokes word value/s into <addr16> (and beyond, if multiple values)" },
  { "poked", cmdPokeD, "<addr16> <dword/s>", "pokes dword value/s into <addr16> (and beyond, if multiple values)" },
  { "pokeq", cmdPokeQ, "<addr16> <qword/s>", "pokes qword value/s into <addr16> (and beyond, if multiple values)" },
  { "mpoke", cmdMPoke, "<addr28> <byte/s>", "pokes byte value/s into <addr28> (and beyond, if multiple values)" },
  { "mpokew", cmdMPokeW, "<addr28> <word/s>", "pokes word value/s into <addr28> (and beyond, if multiple values)" },
  { "mpoked", cmdMPokeD, "<addr28> <dword/s>", "pokes dword value/s into <addr28> (and beyond, if multiple values)" },
  { "mpokeq", cmdMPokeQ, "<addr28> <qword/s>", "pokes qword value/s into <addr28> (and beyond, if multiple values)" },
  { "back", cmdBackTrace, NULL, "produces a rough backtrace from the current contents of the stack" },
  { "up", cmdUpFrame, NULL, "The 'dis' disassembly command will disassemble one stack-level up from the current frame" },
  { "down", cmdDownFrame, NULL, "The 'dis' disassembly command will disassemble one stack-level down from the current frame" },
  { "se", cmdSearch, "<addr28> <len> <values>", "Searches the range you specify for the given values (either a list of hex bytes or a \"string\""},
  { "ss", cmdScreenshot, NULL, "Takes an ascii screenshot of the mega65's screen" },
  { "ty", cmdType, "[<string>]", "Remote keyboard mode (if optional string provided, acts as one-shot message with carriage-return)" },
  { "ftp", cmdFtp, NULL, "FTP access to SD-card" },
  { "petscii", cmdPetscii, "0/1", "In dump commands, respect petscii screen codes" },
  { "fastmode", cmdFastMode, "0/1", "Used to quickly switch between 2,000,000bps (slow-mode: default) or 4,000,000bps (fast-mode: used in ftp-mode)" },
  { "scope", cmdScope, "<int>", "the scope-size of the listing to show alongside the disassembly" },
  { "offs", cmdOffs, "<int>", "the offset of the listing to show alongside the disassembly" },
  { "val", cmdPrintValue, "<hex/#dec/\%bin/>", "print the given value in hex, decimal and binary" },
  { "=", cmdForwardDis, "[<count>]", "move forward in disassembly of pc history from 'z' command" },
  { "-", cmdBackwardDis, "[<count>]", "move backward in disassembly of pc history from 'z' command" },
  { "mcopy", cmdMCopy, "<src_addr> <dest_addr> <count>", "copy data from source location to destination (28-bit addresses)" },
  { "locals", cmdLocals, NULL, "Print out the values of any local variables within the current c-function (needs gurce's cc65 .list file)" },
  { "autolocals", cmdAutoLocals, "0/1", "If set to 1, shows all locals prior to every step/next/dis command" },
  { "mapping", cmdMapping, NULL, "Summarise the current $D030/MAP/$01 mapping of the system" },
  { "seam", cmdSeam, "[row][col]", "display attributes of selected SEAM character" },
  { "blist", cmdBasicList, NULL, "list the current basic program" },
  { "sprite", cmdSprite, "<spridx>", "print out the bits of the sprite at the given index\n"
"                 (based on currently selected vicii bank at $dd00)\n"
"                 If the index is in the form $xxxx, it is treated as an absolute memory address." },
  { "char", cmdChar, "<charidx> [<count>]", "print out the bits of the char at the given index\n"
"                 (based on currently selected vicii bank at $dd00)\n"
"                 If the index is in the form $xxxx, it is treated as an absolute memory address." },
  { "set", cmdSet, "<addr> <string|bytes>", "set bytes at the given address to the desired string or bytes" },
  { "reload", cmdReload, NULL, "reloads any list and map files (in-case you've rebuilt them recently)" },
  { "go", cmdGo, "<addr>", "sets the PC to the desired address." },
  { "palette", cmdPalette, "<startidx> <endidx>", "Shows details of the palette for the given range. If no range given, the first 32 colour indices are selected." },
  { "hyppo", cmdHyppo, "<servicename>", "Performs the desired hyppo call. Note that in many cases, you will have to prepare inputs prior to this call, and assess outputs after the call." },
  { NULL, NULL, NULL, NULL }
};

char* get_extension(char* fname)
{
  return strrchr(fname, '.');
}

int prior_offset = 0; // to help keep track of stack offsets of local variables within functions

typedef struct tli
{
  char* name;
  int offset;
  int size;
  struct tli *next;
} type_localinfo;

typedef struct tfi
{
  char* name;
  int addr;
  type_localinfo* locals;
  int paramsize;
  struct tfi *next;
} type_funcinfo;

type_funcinfo* lstFuncInfo = NULL;
type_funcinfo* cur_func_info = NULL;

typedef struct tcili
{
  char* chunk_name;
  char* section;
  int loc_start;
  int loc_end;
  int size;
  struct tcili* next;
} type_ci_chunk_info;

type_ci_chunk_info* lstCalypsiChunkInfo = NULL;

typedef struct tfl
{
  int addr;
  int lastaddr;
  char* file;
  char* module;
  int lineno;
  struct tfl *next;
} type_fileloc;

type_fileloc* lstFileLoc = NULL;

type_fileloc* cur_file_loc = NULL;

type_symmap_entry* lstSymMap = NULL;

type_offsets segmentOffsets = {{ 0 }};

type_offsets* lstModuleOffsets = NULL;

type_watch_entry* lstWatches = NULL;

void clearSoftBreak(void);
int isCpuStopped(void);
void setSoftBreakpoint(int addr);
void step(void);

void add_to_offsets_list(type_offsets mo)
{
  type_offsets* iter = lstModuleOffsets;
  mo.enabled = 1;

  if (iter == NULL)
  {
    lstModuleOffsets = malloc(sizeof(type_offsets));
    memcpy(lstModuleOffsets, &mo, sizeof(type_offsets));
    lstModuleOffsets->next = NULL;
    return;
  }

  while (iter != NULL)
  {
    //printf("iterating %s\n", iter->modulename);
    // add to end?
    if (iter->next == NULL)
    {
      type_offsets* mo_new = malloc(sizeof(type_offsets));
      memcpy(mo_new, &mo, sizeof(type_offsets));
      mo_new->next = NULL;
      //printf("adding %s\n\n", mo_new->modulename);

      iter->next = mo_new;
      return;
    }
    iter = iter->next;
  }
}

void add_to_locals(type_funcinfo *fi, type_localinfo *li)
{
  type_localinfo* previter = 0;
  type_localinfo* iter = fi->locals;

  if (fi->locals == NULL)
  {
    fi->locals = li;
    return;
  }

  while (iter != NULL)
  {
    // insert entry?
    if (strcmp(iter->name, fi->name) < 0) {
      if (previter == 0) {
        fi->locals = li;
        fi->locals->next = iter;
        return;
      }
      else {
        previter->next = li;
        li->next = iter;
        return;
      }
    }
    if (iter->next == NULL) {
      iter->next = li;
      return;
    }

    previter = iter;
    iter = iter->next;
  }
}

void add_to_func_list(type_funcinfo *fi)
{
  type_funcinfo* previter = 0;
  type_funcinfo* iter = lstFuncInfo;

  cur_func_info = fi;

  if (lstFuncInfo == NULL)
  {
    lstFuncInfo = fi;
    return;
  }

  while (iter != NULL)
  {
    // insert entry?
    if (iter->addr > fi->addr)
    {
      if (previter == 0) {
        lstFuncInfo = fi;
        lstFuncInfo->next = iter;
        return;
      }
      else {
        previter->next = fi;
        fi->next = iter;
        return;
      }
    }
    if (iter->next == NULL) {
      iter->next = fi;
      return;
    }

    previter = iter;
    iter = iter->next;
  }
}

type_ci_chunk_info* add_calypsi_chunk_info(char* chunk_name, char* section, int loc_start, int loc_end, int size)
{
  type_ci_chunk_info* iter = lstCalypsiChunkInfo;

  if (lstCalypsiChunkInfo == NULL) {
    lstCalypsiChunkInfo = malloc(sizeof(type_ci_chunk_info));
    lstCalypsiChunkInfo->chunk_name = strdup(chunk_name);
    lstCalypsiChunkInfo->section = strdup(section);
    lstCalypsiChunkInfo->loc_start = loc_start;
    lstCalypsiChunkInfo->loc_end = loc_end;
    lstCalypsiChunkInfo->size = size;
    lstCalypsiChunkInfo->next = NULL;
    return lstCalypsiChunkInfo;
  }

  while (iter != NULL) {
    if (iter->next == NULL)
    {
      type_ci_chunk_info* cinew = malloc(sizeof(type_ci_chunk_info));
      cinew->chunk_name = strdup(chunk_name);
      cinew->section = strdup(section);
      cinew->loc_start = loc_start;
      cinew->loc_end = loc_end;
      cinew->size = size;
      cinew->next = NULL;

      iter->next = cinew;
      return cinew;
    }
    iter = iter->next;
  }
  return NULL;
}

type_fileloc* add_to_list(type_fileloc fl)
{
  type_fileloc* iter = lstFileLoc;

  // first entry in list?
  if (lstFileLoc == NULL)
  {
    lstFileLoc = malloc(sizeof(type_fileloc));
    lstFileLoc->addr = fl.addr;
    lstFileLoc->lastaddr = fl.lastaddr;
    lstFileLoc->file = strdup(fl.file);
    lstFileLoc->lineno = fl.lineno;
    lstFileLoc->module = fl.module;
    lstFileLoc->next = NULL;
    return lstFileLoc;
  }

  while (iter != NULL)
  {
    // replace existing?
    if (iter->addr == fl.addr)
    {
      iter->file = strdup(fl.file);
      iter->lineno = fl.lineno;
      return iter;
    }
    // insert entry?
    if (iter->addr > fl.addr)
    {
      type_fileloc* flcpy = malloc(sizeof(type_fileloc));
      flcpy->addr = iter->addr;
      flcpy->lastaddr = iter->lastaddr;
      flcpy->file = iter->file;
      flcpy->lineno = iter->lineno;
      flcpy->module = iter->module;
      flcpy->next = iter->next;

      iter->addr = fl.addr;
      iter->lastaddr = fl.lastaddr;
      iter->file = strdup(fl.file);
      iter->lineno = fl.lineno;
      iter->module = fl.module;
      iter->next = flcpy;
      return iter;
    }
    // add to end?
    if (iter->next == NULL)
    {
      type_fileloc* flnew = malloc(sizeof(type_fileloc));
      flnew->addr = fl.addr;
      flnew->lastaddr = fl.lastaddr;
      flnew->file = strdup(fl.file);
      flnew->lineno = fl.lineno;
      flnew->module = fl.module;
      flnew->next = NULL;

      iter->next = flnew;
      return flnew;
    }

    iter = iter->next;
  }

  return NULL;
}

void add_to_symmap(type_symmap_entry sme)
{
  type_symmap_entry* iter = lstSymMap;

  // first entry in list?
  if (lstSymMap == NULL)
  {
    lstSymMap = malloc(sizeof(type_symmap_entry));
    lstSymMap->addr = sme.addr;
    lstSymMap->sval = strdup(sme.sval);
    lstSymMap->symbol = strdup(sme.symbol);
    lstSymMap->next = NULL;
    return;
  }

  while (iter != NULL)
  {
    // insert entry?
    if (iter->addr >= sme.addr)
    {
      type_symmap_entry* smecpy = malloc(sizeof(type_symmap_entry));
      smecpy->addr = iter->addr;
      smecpy->sval = iter->sval;
      smecpy->symbol = iter->symbol;
      smecpy->next = iter->next;

      iter->addr = sme.addr;
      iter->sval = strdup(sme.sval);
      iter->symbol = strdup(sme.symbol);
      iter->next = smecpy;
      return;
    }
    // add to end?
    if (iter->next == NULL)
    {
      type_symmap_entry* smenew = malloc(sizeof(type_symmap_entry));
      smenew->addr = sme.addr;
      smenew->sval = strdup(sme.sval);
      smenew->symbol = strdup(sme.symbol);
      smenew->next = NULL;

      iter->next = smenew;
      return;
    }

    iter = iter->next;
  }
}

void copy_watch(type_watch_entry* dest, type_watch_entry* src)
{
  dest->type = src->type;
  dest->show_decimal = src->show_decimal;
  dest->show_char = src->show_char;
  dest->show_binary = src->show_binary;
  dest->name = strdup(src->name);
  dest->param1 = src->param1 ? strdup(src->param1) : NULL;
  dest->next = NULL;
}

void add_to_watchlist(type_watch_entry we)
{
  type_watch_entry* iter = lstWatches;

  // first entry in list?
  if (lstWatches == NULL)
  {
    lstWatches = malloc(sizeof(type_watch_entry));
    copy_watch(lstWatches, &we);
    return;
  }

  while (iter != NULL)
  {
    // add to end?
    if (iter->next == NULL)
    {
      type_watch_entry* wenew = malloc(sizeof(type_watch_entry));
      copy_watch(wenew, &we);
      iter->next = wenew;
      return;
    }

    iter = iter->next;
  }
}

type_fileloc* find_in_list(int addr)
{
  type_fileloc* iter = lstFileLoc;

  while (iter != NULL)
  {
    //printf("%s : addr=$%04X : line=%d\n", iter->file, iter->addr, iter->lineno);
    if (iter->addr == addr)
      return iter;

    if (iter->lastaddr != 0 && (iter->addr < addr && addr <= iter->lastaddr))
      return iter;

    iter = iter->next;
  }

  return NULL;
}

int find_addr_in_list(char* file, int line)
{
  type_fileloc* iter = lstFileLoc;

  while (iter != NULL)
  {
    if (!strcmp(file, iter->file) && iter->lineno == line)
      return iter->addr;

    iter = iter->next;
  }

  return -1;
}

type_fileloc* find_lineno_in_list(int lineno)
{
  type_fileloc* iter = lstFileLoc;

  if (!cur_file_loc)
    return NULL;

  while (iter != NULL)
  {
    if (strcmp(cur_file_loc->file, iter->file) == 0 && iter->lineno == lineno)
      return iter;

    iter = iter->next;
  }

  return NULL;
}

type_symmap_entry* find_in_symmap(char* sym)
{
  type_symmap_entry* iter = lstSymMap;

  while (iter != NULL)
  {
    if (strcmp(sym, iter->symbol) == 0)
      return iter;

    iter = iter->next;
  }

  return NULL;
}

type_watch_entry* find_in_watchlist(type_watch type, char* name)
{
  type_watch_entry* iter = lstWatches;

  while (iter != NULL)
  {
    if (strcmp(iter->name, name) == 0 && type == iter->type)
      return iter;

    iter = iter->next;
  }

  return NULL;
}

void free_watch(type_watch_entry* iter, int wnum)
{
  free(iter->name);
  if (iter->param1)
    free(iter->param1);
  free(iter);
  if (outputFlag)
    printf("watch#%d deleted!\n", wnum);
}

bool delete_from_watchlist(int wnum)
{
  int cnt = 0;

  type_watch_entry* iter = lstWatches;
  type_watch_entry* prev = NULL;

  while (iter != NULL)
  {
    cnt++;

    // we found the item to delete?
    if (cnt == wnum)
    {
      // first entry of list?
      if (prev == NULL)
      {
        lstWatches = iter->next;
        free_watch(iter, wnum);
        return true;
      }
      else
      {
        prev->next = iter->next;
        free_watch(iter, wnum);
        return true;
      }
    }

    prev = iter;
    iter = iter->next;
  }

  return false;
}

char* get_string_token(char* p, char* name);

char* get_nth_token(char* p, int n)
{
  static char token[128];

  for (int k = 0; k <= n; k++)
  {
    p = get_string_token(p, token);
    if (p == 0)
      return NULL;
  }

  return token;
}

char* get_string_token(char* p, char* name)
{
  int found_start = 0;
  int idx = 0;

  while (1)
  {
    if (!found_start)
    {
      if (*p == 0 || *p == '\r' || *p == '\n')
        return 0;

      if (*p != ' ' && *p != '\t')
      {
        found_start = 1;
        continue;
      }

      p++;
    }

    if (found_start) // found start of token, now look for end;
    {
      if (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
      {
        name[idx] = 0;
        return ++p;
      }

      name[idx] = *p;
      p++;
      idx++;
    }
  }
}

bool starts_with(const char *str, const char *pre)
{
  return strncmp(pre, str, strlen(pre)) == 0;
}

void parse_ca65_segments(FILE* f, char* line)
{
  char name[128];
  char sval[128];
  int val;
  char *p;

  memset(&segmentOffsets, 0, sizeof(segmentOffsets));

  while (!feof(f))
  {
    fgets(line, 1024, f);

    if (line[0] == '\0' || line[0] == '\r' || line[0] == '\n')
    {
      return;
    }

    p = line;
    if (!(p = get_string_token(p, name)))
      return;

    if (!(p = get_string_token(p, sval)))
      return;

    val = strtol(sval, NULL, 16);
    strcpy(segmentOffsets.segments[segmentOffsets.seg_cnt].name, name);
    segmentOffsets.segments[segmentOffsets.seg_cnt].offset = val;
    segmentOffsets.seg_cnt++;
  }
}

void parse_ca65_modules(FILE* f, char* line)
{
  char name[128];
  char sval[128];
  int val;
  int state = 0;
  char *p;
  type_offsets mo = {{ 0 }};

  while (!feof(f))
  {
    fgets(line, 1024, f);

    if (line[0] == '\0' || line[0] == '\r' || line[0] == '\n')
    {
      if (state == 1)
        add_to_offsets_list(mo);
      return;
    }

    if (strchr(line, ':') != NULL)
    {
      line[(int)(strchr(line, ':') - line)] = '\0';
      if (state == 1)
      {
        add_to_offsets_list(mo);
        memset(&mo, 0, sizeof(mo));
      }
      state = 0;
    }

    switch(state)
    {
      case 0: // get module name
        strcpy(mo.modulename, line);
        state = 1;
        break;

      case 1: // get segment offsets
        p = line;
        if (!(p = get_string_token(p, name)))
          return;

        if (!(p = get_string_token(p, sval)))
          return;

        if (!starts_with(sval, "Offs="))
          return;

        p = sval + 5;
        val = strtol(p, NULL, 16);
        strcpy(mo.segments[mo.seg_cnt].name, name);
        mo.segments[mo.seg_cnt].offset = val;
        mo.seg_cnt++;
        //printf("ca65 module: %s : %s - offs = $%04X\n", mo.modulename, name, val);
    }
  }
}

void parse_ca65_symbols(FILE* f, char* line)
{
  char name[128];
  char sval[128];
  int  val;
  char str[64];

  while (!feof(f))
  {
    fgets(line, 1024, f);

    //if (starts_with(line, "zerobss"))
    //  printf(line);

    char* p = line;
    for (int k = 0; k < 2; k++)
    {
      if (!(p = get_string_token(p, name)))
        return;

      p = get_string_token(p,sval);
      val = strtol(sval, NULL, 16);

      p = get_string_token(p,str); // ignore this 3rd one...

      type_symmap_entry sme;
      sme.addr = val;
      sme.sval = sval;
      sme.symbol = name;
      add_to_symmap(sme);
    }
  }
}

void load_ca65_map(FILE* f)
{
  char line[1024];
  rewind(f);
  while (!feof(f))
  {
    fgets(line, 1024, f);

    if (starts_with(line, "Modules list:"))
    {
      fgets(line, 1024, f); // ignore following "----" line
      parse_ca65_modules(f, line);
      continue;
    }
    if (starts_with(line, "Segment list:"))
    {
      fgets(line, 1024, f); // ignore following "----" line
      fgets(line, 1024, f); // ignore following "Name" line
      fgets(line, 1024, f); // ignore following "----" line
      parse_ca65_segments(f, line);
    }
    if (starts_with(line, "Exports list by name:"))
    {
      fgets(line, 1024, f); // ignore following "----" line
      parse_ca65_symbols(f, line);
      continue;
    }
  }
}

void load_lbl(const char* fname)
{
  // load the map file
  FILE* f = fopen(fname, "rt");

  while (!feof(f))
  {
    char line[1024];
    fgets(line, 1024, f);

    char saddr[128];
    char sym[1024];
    sscanf(line, "al %s %s", saddr, sym);

    type_symmap_entry sme;
    sme.addr = get_sym_value(saddr);
    sme.sval = saddr;
    sme.symbol = sym;
    add_to_symmap(sme);
    // printf("%s : %s\n", sme.sval, sym);
  }
  fclose(f);
}

// loads the *.map file corresponding to the provided *.list file (if one exists)
void load_map(const char* fname)
{
  char strMapFile[200];
  strcpy(strMapFile, fname);
  char* sdot = strrchr(strMapFile, '.');
  *sdot = '\0';
  strcat(strMapFile, ".map");

  // check if file exists
  if (access(strMapFile, F_OK) != -1)
  {
    printf("Loading \"%s\"...\n", strMapFile);

    // load the map file
    FILE* f = fopen(strMapFile, "rt");
    int first_line = 1;

    while (!feof(f))
    {
      char line[1024];
      char sval[256];
      fgets(line, 1024, f);

      if (first_line)
      {
        first_line = 0;
        if (starts_with(line, "Modules list:"))
        {
          load_ca65_map(f);
          break;
        }
      }

      int addr;
      char sym[1024];
      sscanf(line, "$%X | %s", &addr, sym);
      sscanf(line, "%s |", sval);

      //printf("%s : %04X\n", sym, addr);
      type_symmap_entry sme;
      sme.addr = addr;
      sme.sval = sval;
      sme.symbol = sym;
      add_to_symmap(sme);
    }
    fclose(f);
  }
}

int get_segment_offset(const char* current_segment)
{
  for (int k = 0; k < segmentOffsets.seg_cnt; k++)
  {
    if (strcmp(current_segment, segmentOffsets.segments[k].name) == 0)
    {
      return segmentOffsets.segments[k].offset;
    }
  }
  return 0;
}

int get_module_offset(const char* current_module, const char* current_segment)
{
  type_offsets* iter = lstModuleOffsets;

  while (iter != NULL)
  {
    if (strcmp(current_module, iter->modulename) == 0)
    {
      for (int k = 0; k < iter->seg_cnt; k++)
      {
        if (strcmp(current_segment, iter->segments[k].name) == 0)
        {
          return iter->segments[k].offset;
        }
      }
    }
    iter = iter->next;
  }
  return 0;
}

char* get_module_string(const char* current_module)
{
  type_offsets* iter = lstModuleOffsets;

  while (iter != NULL)
  {
    if (strcmp(current_module, iter->modulename) == 0)
    {
      return iter->modulename;
    }
    iter = iter->next;
  }
  return 0;
}

void parse_function(char* line, int addr)
{
  char *p = get_nth_token(line, 2);
  if (p != NULL && strcasecmp(p, ".proc") == 0)
  {
    char* p = get_nth_token(line, 3);
    p[strlen(p)-1] = '\0';
    type_funcinfo* fi = malloc(sizeof(type_funcinfo));
    fi->name = strdup(p);
    fi->addr = addr;
    fi->locals = NULL;
    fi->paramsize = 0;
    fi->next = NULL;
    add_to_func_list(fi);
    prior_offset = 0;
  }
}

void parse_debug(char* line)
{
  if (cur_func_info == NULL)
    return;

  char *p1 = get_nth_token(line, 2);
  if (p1 == NULL)
    return;

  if (strcasecmp(p1, ".dbg") == 0)
  {
    char *p2 = get_nth_token(line, 3);
    if (p2 == NULL)
      return;

    if (strcasecmp(p2, "sym,") == 0)
    {
      char* type = get_nth_token(line, 6);
      if (strcasecmp(type, "auto,") == 0)
      {
        char* name = get_nth_token(line, 4);
        name[strlen(name)-2] = '\0';  // trim the end quote and comma

        type_localinfo* li = malloc(sizeof(type_localinfo));
        li->name = strdup(name+1);  // skip the start quote

        char* offset = get_nth_token(line, 7);
        li->offset = atoi(offset);
        li->size = prior_offset - li->offset;
        prior_offset = li->offset;
        li->next = NULL;

        if (strcmp("__sptop__", li->name) == 0) {
          free(li->name);
          cur_func_info->paramsize = li->offset;
          free(li);
          return;
        }

        add_to_locals(cur_func_info, li);
      }
    }
  }
}

void load_ca65_list(const char* fname, FILE* f)
{
  static char list_file_name[256];
  strcpy(list_file_name, fname); // preserve a copy of this eternally

  load_map(fname); // load the ca65 map file first, as it contains details that will help us parse the list file

  char line[1024];
  char current_module[256] = { 0 };
  char current_segment[256] = { 0 };
  int lineno = 1;
  char *cmod = NULL;

  while (!feof(f))
  {
    lineno++;
    fgets(line, 1024, f);

    if (starts_with(line, "Current file:"))
    {
      // Retrieve the current file/module that was assembled
      if (strchr(line, '/') != NULL)  // truncate a relative location like 'src/utilities/remotesd.s'?
      {
        strcpy(current_module, strrchr(line, '/') + 1);
      }
      else
        strcpy(current_module, strchr(line, ':') + 2);
      current_module[strlen(current_module)-1] = '\0';
      current_module[strlen(current_module)-1] = 'o';
      current_segment[0] = '\0';
      // printf("current_module=%s\n", current_module);
      cmod = get_module_string(current_module);
    }

    if (line[0] == '\0' || line[0] == '\r' || line[0] == '\n')
      continue;

    // new .segment specified in code?
    char *p = get_nth_token(line, 2);
    if (p != NULL && strcasecmp(p, ".segment") == 0)
    {
      char* p = get_nth_token(line, 3);
      strncpy(current_segment, p+1, strlen(p+1)-1);
      current_segment[strlen(p+1) - 1] = '\0';
      //if (strcmp(current_module, "fdisk_fat32.o") == 0)
        //printf("line: %s\ncurrent_segment=%s\n", line, current_segment);
      //if (strcmp(current_segment, "CODET") == 0)
        //exit(0);
    }

    // did we find a line with a relocatable address at the start of it
    if (line[0] != ' ' && line[1] != ' ' && line[2] != ' ' && line[3] != ' ' && line[4] != ' ' && line[5] != ' '
        && (line[6] == 'r' || line[6] == ' ') && line[7] == ' ' && line[8] != ' ')
    {
      char saddr[8];
      int addr;
      strncpy(saddr, line, 6);
      saddr[7] = '\0';
      addr = strtol(saddr, NULL, 16);

      // convert relocatable address into absolute address
      if (line[6] == 'r')
      {
        addr += get_segment_offset(current_segment);
        addr += get_module_offset(current_module, current_segment);
      }

      parse_function(line, addr);

      parse_debug(line);

      //if (strcmp(current_module, "fdisk_fat32.o") == 0 /*&& strcmp(current_segment, "CODE") == 0 */ && addr <= 0x1000)
        //printf("mod=%s:seg=%s : %08X : %s", current_module, current_segment, addr, line);
      type_fileloc fl = { 0 };
      fl.addr = addr;
      fl.lastaddr = 0;
      fl.file = list_file_name;
      fl.module = cmod;
      fl.lineno = lineno;
      add_to_list(fl);
    }
  }
}

// loads the given *.list file
void load_list(char* fname)
{
  FILE* f = fopen(fname, "rt");
  char line[1024];
  int first_line = 1;

  while (fgets(line, 1024, f) != NULL)
  {
    if (first_line)
    {
      first_line = 0;
      if (starts_with(line, "ca65"))
      {
        load_ca65_list(fname, f);
        fclose(f);
        return;
      }
    }

    if (strlen(line) == 0)
      continue;

    char *s = strrchr(line, '|');
    if (s != NULL && *s != '\0')
    {
      s++;
      if (strlen(s) < 5)
        continue;

      int addr;
      char file[1024];
      int lineno;
      strcpy(file, &strtok(s, ":")[1]);
      if (strrchr(line, '/'))
        strcpy(file, strrchr(file, '/') + 1);
      sscanf(strtok(NULL, ":"), "%d", &lineno);
      sscanf(line, " %X", &addr);

      //printf("%04X : %s:%d\n", addr, file, lineno);
      type_fileloc fl = { 0 };
      fl.addr = addr;
      fl.lastaddr = 0;
      fl.file = file;
      fl.lineno = lineno;
      fl.module = NULL;
      add_to_list(fl);
    }
  }
  fclose(f);

  load_map(fname);
}

int is_hexc(char c)
{
  if ((c>='a' && c<='z') || (c >='0' && c<='9'))
    return true;

  return false;
}

void load_bsa_list(char* fname)
{
  FILE* f = fopen(fname, "rt");
  char line[1024];
  int lineno=0;

  while (!feof(f))
  {
    fgets(line, 1024, f);
    lineno++;

    if (strlen(line) < 8)
      continue;

    if (is_hexc(line[0]) && is_hexc(line[1]) &&
        is_hexc(line[2]) && is_hexc(line[3]) &&
        line[4] == ' ')
    {
      if (is_hexc(line[5]) && is_hexc(line[6]))
      {
        type_fileloc fl = { 0 };
        int addr;
        sscanf(line, "%X", &addr);
        fl.addr = addr;
        fl.lastaddr = 0;
        fl.file = fname;
        fl.lineno = lineno;
        fl.module = NULL;
        add_to_list(fl);
      }
      else
      {
        char tok[256];
        sscanf(line+5, "%s", tok);
        if (tok[0] != '*')
        {
          // add to map?
          type_symmap_entry sme;
          int addr;
          char sval[256];
          sscanf(line, "%X", &addr);
          sprintf(sval, "$%04X", addr);
          sme.addr = addr;
          sme.sval = sval;
          sme.symbol = tok;
          add_to_symmap(sme);
          // printf("%s : %s ($%04X) : %s\n", line, sval, addr, sme.symbol);
        }
      }
    }
  }
}

void load_acme_map(const char* fname)
{
  char strMapFile[200];
  strcpy(strMapFile, fname);
  char* sdot = strrchr(strMapFile, '.');
  *sdot = '\0';
  strcat(strMapFile, ".sym");

  // check if file exists
  if (access(strMapFile, F_OK) != -1)
  {
    printf("Loading \"%s\"...\n", strMapFile);

    // load the map file
    FILE* f = fopen(strMapFile, "rt");

    while (!feof(f))
    {
      char line[1024];
      char sval[256];
      int addr;
      char sym[1024];
      fgets(line, 1024, f);
      sscanf(line, "%s = %s", sym, sval);
      sscanf(sval, "$%04X", &addr);

      //printf("%s : %04X\n", sym, addr);
      type_symmap_entry sme;
      sme.addr = addr;
      sme.sval = sval;
      sme.symbol = sym;
      add_to_symmap(sme);
    }
    fclose(f);
  }
}

bool is_hex(const char* str)
{
  char c;
  for (int k = 0; k < strlen(str); k++)
  {
    c = str[k];
    if ((c >= 'a' && c <= 'f') || (c >= '0' && c <= '9'))
      continue;
    else
      return false;
  }

  return true;
}

bool is_hexarray(const char* str)
{
  char c;
  int dotcnt = 0;
  for (int k = 0; k < strlen(str); k++)
  {
    c = str[k];
    if ((c >= 'a' && c <= 'f') || (c >= '0' && c <= '9') || c == '.') {
      if (c == '.') {
        dotcnt++;
        if (dotcnt == 3)
          return true;
      }
      continue;
    }
    else
      return false;
  }

  return false;
}

void load_acme_list(char* fname)
{
  FILE* f = fopen(fname, "rt");
  char line[1024];
  char curfile[256] = "";
  int lineno, memaddr;
  char val1[256];
  char val2[256];
  char val3[256];

  bool priorWasByteArray = false;
  type_fileloc *prior_fl = NULL;

  while (!feof(f))
  {
    fgets(line, 1024, f);

    if (starts_with(line, "; ****") && strstr(line, "Source:"))
    {
      char* asmname = strstr(line, "Source: ") + strlen("Source: ");
      if (strrchr(asmname, '/'))
        asmname = strrchr(asmname, '/') + 1;
      sscanf(asmname, "%s", curfile);
    }
    else
    {
      if (sscanf(line, "%d %s %s %s", &lineno, val1, val2, val3) == 4)
      {
        if (is_hex(val1) && (is_hex(val2) || is_hexarray(val2)))
        {
          sscanf(val1, "%04X", &memaddr);

          if (priorWasByteArray) {
            prior_fl->lastaddr = memaddr-1;
            priorWasByteArray = false;
          }

          type_fileloc fl = { 0 };
          fl.addr = memaddr;
          fl.lastaddr = 0;
          fl.file = curfile;
          fl.lineno = lineno;
          prior_fl = add_to_list(fl);

          if (is_hexarray(val2) || val3[0] == '+') // assess acme macros too
            priorWasByteArray = true;
        }
      }
    }
  }
  fclose(f);

  load_acme_map(fname);
}

void show_pixel_block(bool bgflag, int clridx, unsigned char* palette)
{
  unsigned int r = palette[3*clridx + 0];
  unsigned int g = palette[3*clridx + 1];
  unsigned int b = palette[3*clridx + 2];
  if (bgflag) {
    printf("%s.", KNRM);
  } else {
    printf(KINV "\x1b[38;2;%d;%d;%dm ", r, g, b);
  }
}

void assess_ncm_nibble(int clridx, bool bgflag, unsigned char* palette, int foreclr, int extra_clr)
{
  if (clridx == 0x00) {
    // todo: switch to background colour from $d020?
    bgflag = true;
  }
  if (clridx == 0x0f) { // switch to foreground colour?
    clridx = foreclr;
  }
  clridx += extra_clr;
  show_pixel_block(bgflag, clridx, palette);
}

void print_seam_char(unsigned char* palette, int chrnum, int ncm_flag, int foreclr, int extra_clr)
{
  mem_data* mem = get_mem28array(chrnum * 64);

  int idx=0;
  printf(KINV);
  for (int r = 0; r < 16; r++) {
    for (int c = 0; c < 16; c++) {
      bool bgflag = false;
      int clridx = mem[r].b[c];
      if (ncm_flag) {
        assess_ncm_nibble(clridx & 0x0f, bgflag, palette, foreclr, extra_clr);
        assess_ncm_nibble(clridx >> 4, bgflag, palette, foreclr, extra_clr);
      }
      else { // fcm
        if (clridx == 0x00) { // background clr?
          bgflag = true;
        }
        if (clridx == 0xff) {
          clridx = foreclr + extra_clr;
        }
        show_pixel_block(bgflag, clridx, palette);
      }
      idx++;
      if ( (idx % 8) == 0) {
        printf("\n");
      }
    } // end for c
    if (idx == 64)
      break;
  } // end for r
  printf(KNRM);
}


void load_kickass_map(char* fname)
{
  char strMapFile[200];
  strcpy(strMapFile, fname);
  char* sdot = strrchr(strMapFile, '.');
  *sdot = '\0';
  strcat(strMapFile, ".sym");

  // check if file exists
  if (access(strMapFile, F_OK) != -1)
  {
    printf("Loading \"%s\"...\n", strMapFile);

    // load the map file
    FILE* f = fopen(strMapFile, "rt");

    while (!feof(f))
    {
      char line[1024];
      char cpy_line[1024];
      //char sval[256];
      int addr;
      char sym[1024];
      char *token;
      //if ((strlen(line)>7) && (strstr(line, ".label "))) {
      if((fgets(line, 1024, f) != NULL) && (starts_with(line, ".label "))) {
        strcpy(cpy_line,line);

        char* asmname = strstr(line, ".label ") + strlen(".label ");

        token = strtok(asmname, "=");

        strcpy(sym,token);

        char* sval = strstr(cpy_line, "$");

        //printf("%s\n", sval);
        sscanf(sval, "$%04X", &addr);

        //sscanf(sval, "$%04X", &addr);

        //printf("%s : %04X %s \n", sym, addr, cpy_line);

        type_symmap_entry sme;
        sme.addr = addr;
        sme.sval = sval;
        sme.symbol = sym;
        add_to_symmap(sme);
      }
    }
    fclose(f);
  }
}

typedef struct {
  int state;  // 0 = look for 'in section', 1 = look for 'placed at' (retrieve address)
  char* line;
  int lineno;
  char* fname;
  FILE* f;
  bool is_linker_clst_file;
  int cur_srcline;
  int cur_rel_addr;
  char chunk_name[64];
} typ_calypsi_info;

#define STATE_LOOK_SECTION 0
#define STATE_LOOK_PLACED_AT 1
#define STATE_LOOK_FILENAME 2
#define STATE_LOOK_DEFINES 3
#define STATE_READ_DEFINES 4


void check_wonky_label(typ_calypsi_info* ci, char* label)
{
  char filename[64];
  char* fname = filename;

  // truncate filename without extension
  sscanf(ci->line, "(%s ", filename);
  if (strrchr(filename, '/'))
    fname = strrchr(filename, '/')+1;

  char*e = strrchr(fname, '.');
  if (e) {
    *e = '\0';
  }

  // for wonky labels, add filename at start
  char label_name[64];
  strcpy(label_name, label);
  if (label[0] == '`') {
    strcpy(label_name, fname);
    strcat(label_name, ":");
    strcat(label_name, label);

    // copy over original string
    strcpy(label, label_name);
    printf("wonky = %s\n", label);
  }
}

void parse_calypsi_linker_line(typ_calypsi_info* ci)
{
  static char chunk_name[64];
  static char section[64];
  static int loc_start, loc_end;
  static int size;

  switch (ci->state) {

// modplay_reset in section 'code'
 // placed at address 00004b9f-00004c57 of size 000000b9

    case STATE_LOOK_SECTION:   // look for 'in section' and 'placed at'
      if (strstr(ci->line, "in section")) {

        sscanf(ci->line, "%s in section '%s'", chunk_name, section);

        ci->line = strstr(ci->line, "placed at address");
        if (!ci->line)
        {
          ci->state = STATE_LOOK_PLACED_AT;
          return;
        }
        sscanf(ci->line, "placed at address %08x-%08x of size %08x", &loc_start, &loc_end, &size);

        ci->state = STATE_LOOK_FILENAME;
      }
      break;

    case STATE_LOOK_PLACED_AT: // look for 'placed at' carried onto next line
      ci->line = strstr(ci->line, "placed at address");
      if (!ci->line) {
        ci->state = STATE_LOOK_SECTION;
        return;
      }
      sscanf(ci->line, "placed at address %08x-%08x of size %08x", &loc_start, &loc_end, &size);

      ci->state = STATE_LOOK_FILENAME;
      break;

    case STATE_LOOK_FILENAME:
      if (!strstr(ci->line, " section index ")) {
        ci->state = STATE_LOOK_SECTION;
        return;
      }

      check_wonky_label(ci, chunk_name);

      add_calypsi_chunk_info(chunk_name, section, loc_start, loc_end, size);
      ci->state = STATE_LOOK_DEFINES;
      break;

    case STATE_LOOK_DEFINES: // look for 'Defines:'
      if (strstr(ci->line, "Defines:"))
        ci->state = STATE_READ_DEFINES;
      if (strlen(ci->line) == 0)
        ci->state = STATE_LOOK_SECTION;
      if (strlen(ci->line) == 1 && ci->line[0] == '\n')
        ci->state = STATE_LOOK_SECTION;
      break;

    case STATE_READ_DEFINES: // read out all defines
      if (strstr(ci->line, "References:") ||
          strstr(ci->line, "Referenced from:") ||
          strlen(ci->line) == 0 || strlen(ci->line) == 1)
      {
        ci->state = STATE_LOOK_SECTION;
        break;
      }

      char* p = get_nth_token(ci->line, 0);
      printf("define chunk: %s\n", p);

      if (!p) {
        printf("ERROR: %s - line %d\n", ci->fname, ci->lineno);
        printf("ci->line: '%s'\n", ci->line);
      }

      if (strcmp(p, chunk_name) != 0)
        add_calypsi_chunk_info(p, section, loc_start, loc_end, size);
      break;
  }
}

int get_chunk_offset(char* chunk_name) {
  type_ci_chunk_info* iter = lstCalypsiChunkInfo;

  while (iter != NULL)
  {
    if (strcmp(iter->chunk_name, chunk_name) == 0)
    {
      return iter->loc_start;
    }
    iter = iter->next;
  }

  return -1;
}

void add_file_loc(typ_calypsi_info* ci)
{
  type_fileloc fl = { 0 };
  int addr = ci->cur_rel_addr + get_chunk_offset(ci->chunk_name);
  fl.addr = addr;
  fl.lastaddr = 0;
  fl.file = ci->fname;
  fl.lineno = ci->lineno;
  fl.module = NULL;
  add_to_list(fl);
}

void add_label_to_symmap(char* label, int addr, char* fname)
{
  char label_name[64];
  strcpy(label_name, label);

  // add to map?
  type_symmap_entry sme;
  // done earlier now
  // int addr = cur_rel_addr + get_chunk_offset(chunk_name);
  char sval[256];
  sprintf(sval, "$%04X", addr);
  sme.addr = addr;
  sme.sval = sval;
  sme.symbol = label_name;
  add_to_symmap(sme);
}

void check_chunk_offset(typ_calypsi_info* ci, char* label, int* addr)
{
  char label_name[64];
  strcpy(label_name, label);

  // handle the messy `?Lxx` labels
  if (label[0] == '`') {
    char*e = strchr(ci->fname, '.');
    strncpy(label_name, ci->fname, (int)(e - ci->fname));
    label_name[(int)(e - ci->fname)] = '\0';
    strcat(label_name, ":");
    strcat(label_name, label);
  }

  if (get_chunk_offset(label_name) != -1) {
    strcpy(ci->chunk_name, label_name);
    *addr = ci->cur_rel_addr + get_chunk_offset(ci->chunk_name);
  }
}


void find_and_add_label(typ_calypsi_info* ci)
{
  int addr = ci->cur_rel_addr + get_chunk_offset(ci->chunk_name);

  char* p = get_nth_token(ci->line, 4);
  if (p) {
    if (strcmp(p, ".byte") == 0) {
      p = get_nth_token(ci->line, 3);

      check_chunk_offset(ci, p, &addr);
      printf("label: '%s' @ $%04X (from .byte) - line#%d\n", p, addr, ci->lineno);
      add_label_to_symmap(p, addr, ci->fname);
    }
  }

  // look for any labels on this line (ending with ':')
  char* e = strchr(ci->line, ':');
  if (!e)
    return;

  if (e[1] == '\0' || e[1] == ' ' || e[1] == '\t'
      || e[1] == '\r' || e[1] == '\n')
  {
    *e = '\0';
    char* s = strrchr(ci->line, ' ');
    if (s)
    {
      s++;
      if (s[0] == '\t')
        s++;

      check_chunk_offset(ci, s, &addr);

      printf("label: '%s' @ $%04X (from .byte) - line#%d\n", s, addr, ci->lineno);
      add_label_to_symmap(s, addr, ci->fname);
    }
  }
}

bool find_public_chunk_name(typ_calypsi_info* ci)
{
  char* s;
  // look for any .public <chunk_name>
  s = strstr(ci->line, ".public ");
  if (s) {
    s += strlen(".public ");
    if (s[strlen(s)-1] == '\n')
      s[strlen(s)-1] = '\0';
    strcpy(ci->chunk_name, s);

    if (ci->chunk_name[strlen(ci->chunk_name)-1] == '\n')
      ci->chunk_name[strlen(ci->chunk_name)-1] = '\0';

    return true;
  }
  return false;
}

bool find_section_name(typ_calypsi_info* ci)
{
  char* s;
  // look for any .public <chunk_name>
  s = strstr(ci->line, ".section ");
  if (s) {
    ci->cur_rel_addr = 0;
    return true;
  }
  return false;
}

void parse_calypsi_compiler_line(typ_calypsi_info* ci)
{
  int val;
  bool count_bytes = false;

  if (strlen(ci->line) >= 4 && sscanf(ci->line, "%04d", &val) == 1) {
    ci->cur_srcline = val;

    // E.g.: '0010                		.public audio_applyvolume'
    // E.g.: '0283                romloaded:'
    // E.g.: '0314  0218 bd....   carc700:	lda runmeafterreset,x'
//> L84:         0075                             .section code
//> L85: [$6982] 0076                nmi_handler:
//> L86: [$695A] 0077  0000 40                    rti

    if (find_public_chunk_name(ci))
      return;

    if (find_section_name(ci))
      return;

    if (strlen(ci->line) >= 10 &&
        ci->line[4] == ' ' && ci->line[5] == ' ' &&
        ci->line[6] != ' ' && ci->line[7] != ' ' &&
        ci->line[8] != ' ' && ci->line[9] != ' ' &&
        ci->line[10] == ' ' &&
        sscanf(ci->line+6, "%04x", &val) == 1)
    {
      ci->cur_rel_addr = val;
      count_bytes = true;
    }

    find_and_add_label(ci);

    if (count_bytes) {
      add_file_loc(ci);

      int cnt = 0;
      char* b = &ci->line[11];
      while (*b != ' ') {
        cnt++;
        b++;
      }
      ci->cur_rel_addr += cnt/2;
    }
  }
  // E.g.: '    \ 0000                      .public mp_dmacopyjob'
  if (sscanf(ci->line, "    \\ %04X ", &val) == 1) {
    // sometimes a section just follows the one before it
    // so it resets the cur_rel_addr to zero
    // E.g.: (from 'main.clst' - IntroDisk - L#391)
    //       '    \ 01df 4c....               jmp     `?L5`'
    //       '    \ 0000                      .section code,text'
    //       '    \ 0000 a97f     `?L32`:     lda     #127''
    // (this is a case were the label dictates the chunk-name)
    // (maybe like a 'private' chunk?)

    ci->cur_rel_addr = val;

    if (find_public_chunk_name(ci))
      return;

    if (find_section_name(ci))
      return;

    find_and_add_label(ci);

    add_file_loc(ci);
  }
}

char* find_cmap_file(void)
{
  DIR           *d;
  struct dirent *dir;
  static char cmap_file[256];

  d = opendir(".");
  if (d)
  {
    while ((dir = readdir(d)) != NULL)
    {
      char* ext = get_extension(dir->d_name);
      if (ext != NULL && strcmp(ext, ".cmap") == 0) {
        strcpy(cmap_file, dir->d_name);
        return cmap_file;
      }
    }
  }
  return NULL;
}

void load_calypsi_map(const char* fname)
{
  char* strMapFile = find_cmap_file();

  if (strMapFile == NULL) {
    printf("WARNING: Could not find .cmap file for Calypsi debugging\n");
    return;
  }

  typ_calypsi_info ci = { 0 };
  char line[1024];
  ci.lineno = 0;

  // check if file exists
  if (access(strMapFile, F_OK) != -1)
  {
    printf("Loading \"%s\"...\n", strMapFile);

    // load the map file
    ci.f = fopen(strMapFile, "rt");
    ci.fname = strMapFile;

    while (!feof(ci.f))
    {
      fgets(line, 1024, ci.f);
      ci.line = line;
      ci.lineno++;

      parse_calypsi_linker_line(&ci);
    }
  }

  fclose(ci.f);
}

void load_calypsi_list(char* fname)
{
  typ_calypsi_info ci = { 0 };
  char line[1024];

  ci.f = fopen(fname, "rt");
  ci.fname = fname;
  ci.lineno = 0;

  while (fgets(line, sizeof(line), ci.f) != NULL) {
    ci.line = line;
    ci.lineno++;

    parse_calypsi_compiler_line(&ci);
  }

  fclose(ci.f);
}


void load_KickAss_list(char* fname)
{

  FILE* f = fopen(fname, "rt");
  char line[1024];
  int lineno;

  char segment[256]={0}, module[256]={0};
  lineno = 0;

  while (fgets(line, sizeof(line), f)!=NULL) {
      lineno++;
      // Controlla se la riga contiene un indirizzo (assumiamo che inizi con un indirizzo in esadecimale)
      //unsigned int address;
      int addr;
      char *token;

      if (starts_with(line, "****") && strstr(line, "Segment:"))
      {
          char* asmname = strstr(line, "Segment:") + strlen("Segment: ");
          sscanf(asmname, "%s", segment);

          //printf("find the Segment: %s\n", segment);
      }
      else if (starts_with(line, "[") && strstr(line, "]")){
          int len =strstr(line, "]")-line-1;
          strncpy(module, line+1, len);
          module[len] = '\0';

          //printf("find the Module: %s\n", module);
      }
      else if (strstr(line, "-") != NULL){
          // first :
          token = strtok(line, ":");
          //printf("find the address: %s\n", token);
          sscanf(token, "%04X", &addr);

          type_fileloc fl = { 0 };
          fl.addr = addr;
          fl.lastaddr = 0;
          fl.module = NULL;
          fl.file = fname;
          fl.lineno = lineno;
          add_to_list(fl);

          //printf("Lista: addr %d, module %s, file %s, line %d \n", addr, module, fname, lineno);
      }

    }
    fclose(f);
    load_kickass_map(fname);
  }

void show_location(type_fileloc* fl)
{
  FILE* f = fopen(fl->file, "rt");
  if (f == NULL)
    return;
  char line[1024];
  int cnt = 1;

  while (!feof(f))
  {
    fgets(line, 1024, f);
    if (cnt >= (fl->lineno - dis_scope + dis_offs) && cnt <= (fl->lineno + dis_scope + dis_offs) )
    {
      int addr = find_addr_in_list(fl->file, cnt);
      char saddr[16] = "       ";
      if (addr != -1)
        sprintf(saddr, "[$%04X]", addr);

      if (cnt == fl->lineno)
      {
        printf("%s> L%d: %s %s%s", KINV, cnt, saddr, line, KNRM);
      }
      else
        printf("> L%d: %s %s", cnt, saddr, line);
      //break;
    }
    cnt++;
  }
  fclose(f);
}

// search the current directory for *.list files
void listSearch(void)
{
  bool calypsi_map_loaded = false;
  DIR           *d;
  struct dirent *dir;

  d = opendir(".");
  if (d)
  {
    while ((dir = readdir(d)) != NULL)
    {
      char* ext = get_extension(dir->d_name);
      // VICE label file?
      if (ext != NULL && strcmp(ext, ".lbl") == 0)
      {
        printf("Loading \"%s\"...\n", dir->d_name);
        load_lbl(dir->d_name);
      }
      // .klist = KickAss Compiler
      if (ext != NULL && strcmp(ext, ".klist") == 0)
      {
        printf("Loading \"%s\"...\n", dir->d_name);
        load_KickAss_list(dir->d_name);
      }
      if (ext != NULL && strcmp(ext, ".clst") == 0)
      {
        printf("Loading \"%s\"...\n", dir->d_name);
        if (!calypsi_map_loaded) {
          load_calypsi_map(dir->d_name);
          calypsi_map_loaded = true;
        }
        load_calypsi_list(dir->d_name);
      }
      // .lst = BSA Compiler for MEGA65 ROM
      if (ext != NULL && strcmp(ext, ".lst") == 0)
      {
        printf("Loading \"%s\"...\n", dir->d_name);
        load_bsa_list(dir->d_name);
      }
      // .list = Ophis or CA65?
      if (ext != NULL && strcmp(ext, ".list") == 0)
      {
        printf("Loading \"%s\"...\n", dir->d_name);
        load_list(dir->d_name);
      }
      // .rep = ACME (report file, equivalent to .list)
      else if (ext != NULL && strcmp(ext, ".rep") == 0)
      {
        printf("Loading \"%s\"...\n", dir->d_name);
        load_acme_list(dir->d_name);
      }
    }

    closedir(d);
  }
}

reg_data get_regs(void)
{
  reg_data reg = { 0 };
  char* line;
  while (1) {
    serialWrite("r\n");
    serialRead(inbuf, BUFSIZE);
    line = strstr(inbuf+2, "\n");
    if (!line) // did we hit a null+1? try again
      continue;
    line++;
    sscanf(line,"%04X %02X %02X %02X %02X %02X %04X %04X %04X %02X %02X %02X %s",
      &reg.pc, &reg.a, &reg.x, &reg.y, &reg.z, &reg.b, &reg.sp, &reg.maph, &reg.mapl, &reg.lastop, &reg.odd1, &reg.odd2, reg.flags);
    break;
  }

  return reg;
}

void show_regs(reg_data* reg)
{
  printf("PC   A  X  Y  Z  B  SP   MAPH MAPL LAST-OP     P  P-FLAGS   RGP uS IO\n");
  printf("%04X %02X %02X %02X %02X %02X %04X %04X %04X %02X       %02X %02X %s\n", reg->pc, reg->a, reg->x, reg->y, reg->z, reg->b, reg->sp, reg->maph, reg->mapl, reg->lastop, reg->odd1, reg->odd2, reg->flags);
  printf("     ");
  print_char(reg->a);
  printf("  ");
  print_char(reg->x);
  printf("  ");
  print_char(reg->y);
  printf("  ");
  print_char(reg->z);
  printf("\n");
}

mem_data get_mem(int addr, bool useAddr28)
{
  mem_data mem = { 0 };
  char str[100];
  if (useAddr28)
    sprintf(str, "m%07X\n", addr); // use 'm' (for 28-bit memory addresses)
  else
    sprintf(str, "m777%04X\n", addr); // set upper 12-bis to $777xxxx (for memory in cpu context)

  serialWrite(str);
  serialRead(inbuf, BUFSIZE);
  sscanf(inbuf, ":%X:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
  &mem.addr, &mem.b[0], &mem.b[1], &mem.b[2], &mem.b[3], &mem.b[4], &mem.b[5], &mem.b[6], &mem.b[7], &mem.b[8], &mem.b[9], &mem.b[10], &mem.b[11], &mem.b[12], &mem.b[13], &mem.b[14], &mem.b[15]);

  return mem;
}

int peek(unsigned int address)
{
  mem_data mem = get_mem(address, false);
  return mem.b[0];
}

int mpeek(unsigned int address)
{
  mem_data mem = get_mem(address, true);
  return mem.b[0];
}

int mpeekw(unsigned int address)
{
  mem_data mem = get_mem(address, true);
  return mem.b[0] + (mem.b[1] << 8);
}

void poke(unsigned int address, int val)
{
  char command_str[1024] = { 0 };
  char nval[10];

  sprintf(command_str, "s %04X ", address);

  sprintf(nval, "%02X", val);
  strcat(command_str, nval);
  strcat(command_str, " ");

  strcat(command_str, "\n");

  serialWrite(command_str);
  serialRead(inbuf, BUFSIZE);
}

void pokew(unsigned int address, int val)
{
  char command_str[1024] = { 0 };
  char nval[10];

  sprintf(command_str, "s %04X ", address);

  sprintf(nval, "%02X", val & 0xff);
  strcat(command_str, nval);
  strcat(command_str, " ");

  sprintf(nval, "%02X", (val >> 8) & 0xff);
  strcat(command_str, nval);
  strcat(command_str, " ");

  strcat(command_str, "\n");

  serialWrite(command_str);
  serialRead(inbuf, BUFSIZE);
}

// read all 16 at once (to hopefully speed things up for saving memory dumps)
mem_data* get_mem28array(int addr)
{
  static mem_data multimem[32];
  mem_data* mem;
  char str[100];
  sprintf(str, "M%04X\n", addr);
  serialWrite(str);
  serialRead(inbuf, BUFSIZE);
  char* strLine = strtok(inbuf, "\n");
  for (int k = 0; k < 16; k++)
  {
    mem = &multimem[k];
    sscanf(strLine, ":%X:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
    &mem->addr, &mem->b[0], &mem->b[1], &mem->b[2], &mem->b[3], &mem->b[4], &mem->b[5], &mem->b[6], &mem->b[7], &mem->b[8], &mem->b[9], &mem->b[10], &mem->b[11], &mem->b[12], &mem->b[13], &mem->b[14], &mem->b[15]);
    strLine = strtok(NULL, "\n");
  }

  return multimem;
}

int parse_indices(char* str, int* x, int* y)
{
  int state = 0;
  char s[100] = "";
  int i=0;

  for (int k = 0; k < strlen(str); k++)
  {
    switch(state)
    {
      case 0: // expect [
        if (str[k] == '[')
        {
          state = 1;
        }
        else if (str[k] != ' ')
          return 0;
        break;

        // - - - -

      case 1: // expect numeric
        if (str[k] >= '0' && str[k] <= '9') {
          s[i] = str[k];
          i++;
          s[i] = '\0';
        }
        else if (str[k] == ']') {
          *y = atoi(s);
          i=0;
          s[i] = '\0';
          state = 2;
        }
        else if (str[k] != ' ')
          return 0;
        break;

        // - - - -

      case 2:
        if (str[k] == '[')
        {
          state = 3;
        }
        else if (str[k] != ' ')
          return 0;
        break;

        // - - - -

      case 3: // expect numeric
        if (str[k] >= '0' && str[k] <= '9') {
          s[i] = str[k];
          i++;
          s[i] = '\0';
        }
        else if (str[k] == ']') {
          *x = atoi(s);
          i=0;
          s[i] = '\0';
          return 1;
        }
        else if (str[k] != ' ')
          return 0;
        break;
    }
  }

  if (state == 2 && i == 0)
    return 2; // show entire row details

  return 0;
}

#define GOTOX_CLEAR 0
#define GOTOX_SET 1
#define GOTOX_EITHER 2
#define V3_EXT_ATTR 1
#define V2_MCM 2
#define SCR0 0
#define SCR1 1
#define CLR0 2
#define CLR1 3


BitfieldInfo bitfields[] = {
  { "char_number",      GOTOX_CLEAR,  SCR0, 0, 8, SCR1, 0, 5 },
  { "rhs_trim",         GOTOX_CLEAR,  SCR1, 5, 3, CLR0, 2, 1 },
  { "ncm_flag",         GOTOX_CLEAR,  CLR0, 3, 1, -1, -1, -1 },
  { "gotox_flag",       GOTOX_EITHER, CLR0, 4, 1, -1, -1, -1 },
  { "alpha_mode",       GOTOX_CLEAR,  CLR0, 5, 1, -1, -1, -1 },
  { "horz_flip",        GOTOX_CLEAR,  CLR0, 6, 1, -1, -1, -1 },
  { "vert_flip",        GOTOX_CLEAR,  CLR0, 7, 1, -1, -1, -1 },
  { "clr_4bit",         GOTOX_CLEAR,  CLR1, 0, 4, -1, -1, -1 },
  { "blink",            GOTOX_CLEAR | V3_EXT_ATTR, CLR1, 4, 1, -1, -1, -1 },
  { "reverse",          GOTOX_CLEAR | V3_EXT_ATTR, CLR1, 5, 1, -1, -1, -1 },
  { "bold",             GOTOX_CLEAR | V3_EXT_ATTR, CLR1, 6, 1, -1, -1, -1 },
  { "underline",        GOTOX_CLEAR | V3_EXT_ATTR, CLR1, 7, 1, -1, -1, -1 },
  { "clr_8bit",         GOTOX_CLEAR | V2_MCM, CLR1, 0, 8, -1, -1, -1 },
  { "gotox_pos",        GOTOX_SET,    SCR0, 0, 8, SCR1, 0, 2 },
  { "fcm_yoffs_dir",    GOTOX_SET,    SCR1, 4, 1, -1, -1, -1 }, // substract yoffs instead of add
  { "fcm_yoffs",        GOTOX_SET,    SCR1, 5, 3, -1, -1, -1 },
  { "foreground_flag",  GOTOX_SET,    CLR0, 2, 1, -1, -1, -1 },
  { "rowmask_flag",     GOTOX_SET,    CLR0, 3, 1, -1, -1, -1 },
  { "background_flag",  GOTOX_SET,    CLR0, 6, 1, -1, -1, -1 },
  { "transparent_flag", GOTOX_SET,    CLR0, 7, 1, -1, -1, -1 },
  { "rowmask",          GOTOX_SET,    CLR1, 0, 8, -1, -1, -1 },
  { NULL,               -1,           -1, -1, -1, -1, -1, -1 }
};

void print_seam(int addr, int chars_per_row, int mcm_flag, int ext_attrib_flag, int clr_base, int x, int y)
{
  printf("seam[%d][%d]:\n", y, x);

  // read screen word
  int scr_addr = addr + y * (chars_per_row * 2) + (x * 2);
  int clr_addr = 0xff80000 + clr_base + y * (chars_per_row * 2) + (x * 2);

  mem_data mem = get_mem(scr_addr, true);
  int scr0 = mem.b[0];
  int scr1 = mem.b[1];
  mem = get_mem(clr_addr, true);
  int clr0 = mem.b[0];
  int clr1 = mem.b[1];
  printf("  $%08X : scr0 = $%02X : %%%s\n", scr_addr+0, scr0, toBinaryString(scr0, NULL));
  printf("  $%08X : scr1 = $%02X : %%%s\n", scr_addr+1, scr1, toBinaryString(scr1, NULL));
  printf("  $%08X : clr0 = $%02X : %%%s\n", clr_addr+0, clr0, toBinaryString(clr0, NULL));
  printf("  $%08X : clr1 = $%02X : %%%s\n", clr_addr+1, clr1, toBinaryString(clr1, NULL));

  unsigned char* palette = get_palette();

  // check if GOTOX is SET
  if (clr0 & 0x10) {
    printf("\x1b[38;2;255;0;0mGOTOX is SET" KNRM "\n");
    int goto_x = scr0 + ((scr1 & 0x01) << 8);
    if (scr1 & 0x02) // highest bit is a sign bit
      goto_x -= 512;
    printf("  .goto_x = %d\n", goto_x);
    int fcm_yoffs = scr1 >> 5;
    printf("  .fcm_yoffs = %d\n", fcm_yoffs);
    int transparent_flag = clr0 & 0x80 ? 1 : 0;
    int background_flag = clr0 & 0x40 ? 1 : 0;
    int rowmask_flag = clr0 & 0x08 ? 1 : 0;
    int foreground_flag = clr0 & 0x04 ? 1 : 0;
    printf("  .transparent_flag = %d\n", transparent_flag);
    printf("  .background_flag = %d\n", background_flag);
    printf("  .rowmask_flag = %d\n", rowmask_flag);
    printf("  .foreground_flag = %d\n", foreground_flag);

    printf("  .rowmask = %%%s\n", toBinaryString(clr1, NULL));
  }
  // else GOTOX is CLEAR
  else {
    printf("\x1b[38;2;0;0;255mGOTOX is CLEAR" KNRM "\n");
    int chrnum = scr0 + ( (scr1 & 0x1f) << 8);
    printf("  .char_number = %d ($%04X)\n", chrnum, chrnum);
    int rhs_trim = ((scr1 & 0xe0) >> 5) + ((clr0 & 0x04) << 1);
    printf("  .rhs_trim = %d\n", rhs_trim);
    int vert_flip = (clr0 & 0x80) ? 1 : 0;
    int horz_flip = (clr0 & 0x40) ? 1 : 0;
    int alpha_mode = (clr0 & 0x20) ? 1 : 0;
    int ncm_flag = (clr0 & 0x08) ? 1 : 0;
    printf("  .vert_flip = %d\n", vert_flip);
    printf("  .horz_flip = %d\n", horz_flip);
    printf("  .alpha_mode = %d\n", alpha_mode);
    printf("  .ncm_flag = %d\n", ncm_flag);

    int extra_clr = 0;
    //if (mcm_flag) {
      extra_clr = (clr1 & 0xf0);
      printf("MCM: extra_clr = $%02X\n", extra_clr);
    //}
    /*else*/ if (ext_attrib_flag) {
      int underline = (clr1 & 0x80) ? 1 : 0;
      int bold = (clr1 & 0x40) ? 1 : 0;
      int reverse = (clr1 & 0x20) ? 1 : 0;
      int blink = (clr1 & 0x10) ? 1 : 0;
      printf("EXT_ATTR: underline=%d, bold=%d, reverse=%d, blink=%d\n",
          underline, bold, reverse, blink);
      if (bold && reverse)
        printf("  (bold+reverse = alt. palette)\n");
    }
    int clr = clr1 & 0x0f;
    printf("clr = $%02X\n", clr);
    printf("\n");
    print_seam_char(palette, chrnum, ncm_flag, clr, extra_clr);
  }
  printf("\n");

}

char* find_break_char(char *s, char *tokens) {
  char *t = tokens;
  char *f = NULL;
  while (*t != '\0') {
    f = strchr(s, *t);
    if (f != NULL)
      return f;
    t++;
  }
  return NULL;
}

int find_bitfield_by_name(char *name)
{
  int k = 0;
  while (bitfields[k].name != NULL) {
    if (strcmp(name, bitfields[k].name) == 0) {
      return k;
    }
    k++;
  }
  return -1;
}


void set_field(int* mem, int start_bit, int num_bits, int value)
{
    // Create a mask with the specified bit range set to 1
    int mask = ((1 << num_bits) - 1) << start_bit;

    // Clear the bitfield in the original value (bitwise AND with the inverse of the mask)
    *mem &= ~mask;

    // Shift the value into the correct position (align it to the starting bit)
    value &= (1 << num_bits) - 1; // Ensure value fits within the specified bit width
    *mem |= value << start_bit; // Set the value in the specified bitfield
}


void check_if_setting_field(int addr, int chars_per_row, int clr_base, char *str, int x, int y)
{
  char *start, *end;

  char *s = strrchr(str, ']');
  s++;
  if (*s != '.')
    return;

  s++;
  start = s;
  end = find_break_char(start, " =");
  if (end == NULL)
    return;

  bool eq_found = false;
  if (*end == '=')
    eq_found = true;

  *end = '\0';

  int bf = find_bitfield_by_name(start);
  if (bf == -1) {
    printf("ERROR: Unable to find seam bitfield name\n");
    return;
  }

  if (!eq_found) {
    s = end + 1;
    end = strchr(s, '=');
    if (end == NULL) {
      printf("ERROR: '=' not found (TODO: print value of field instead)\n");
      return;
    }
  }

  s = end + 1;

  int value = get_sym_value(s);

  // read screen word
  int scr_addr = addr + y * (chars_per_row * 2) + (x * 2);
  int clr_addr = 0xff80000 + clr_base + y * (chars_per_row * 2) + (x * 2);

  mem_data mem = get_mem(scr_addr, true);
  int bytes[4];
  bytes[SCR0] = mem.b[0];
  bytes[SCR1] = mem.b[1];
  mem = get_mem(clr_addr, true);
  bytes[CLR0] = mem.b[0];
  bytes[CLR1] = mem.b[1];

  // apply segment 1
  set_field(&bytes[bitfields[bf].byte_index1], bitfields[bf].start_bit1, bitfields[bf].num_bits1, value);
  if (bitfields[bf].byte_index2 != -1)
    set_field(&bytes[bitfields[bf].byte_index2], bitfields[bf].start_bit2, bitfields[bf].num_bits2, value >> bitfields[bf].num_bits1);

  // poke it back to memory
  sprintf(str, "s%X %x %x\n", scr_addr, bytes[SCR0], bytes[SCR1]);
  serialWrite(str);
  serialRead(inbuf, BUFSIZE);

  sprintf(str, "s%X %x %x\n", clr_addr, bytes[CLR0], bytes[CLR1]);
  serialWrite(str);
  serialRead(inbuf, BUFSIZE);
}


void cmdSeam(void)
{
  // get screen ram address at $D060-D063
  mem_data mem = get_mem(0xffd305e, true);
  int addr = mem.b[2] + (mem.b[3] << 8) + (mem.b[4] << 16)
    + ( (mem.b[5] & 0x0f) << 24);
  printf("$D060-$D063.0-3: screen address = $%08X\n", addr);
  int chars_per_row = mem.b[0] + ( (mem.b[5] & 0x30) << 4);
  printf("$D05E+$D063.4-5: chars per row = %d\n", chars_per_row);

  int reg_d054 = mpeek(0xffd3054);
  int chr16 = reg_d054 & 1;
  int fclrlo = reg_d054 & 2 ? 1 : 0;
  int fclrhi = reg_d054 & 4 ? 1 : 0;
  int clr_base = mpeek(0xffd3064) + (mpeek(0xffd3065) << 8);

  printf("$D054.0: CHR16 = %d\n", chr16);
  printf("$D054.1: FCLRLO = %d\n", fclrlo);
  printf("$D054.2: FCLRHI = %d\n", fclrhi);
  printf("$D064-$D065: COLPTR = $%04X\n", clr_base);

  int mcm_flag = (mpeek(0xffd3016) & 0x10) ? 1 : 0;
  printf("$D016.4: mcm_flag = %d\n", mcm_flag);
  int ext_attrib_flag = (mpeek(0xffd3031) & 0x20) ? 1 : 0;
  printf("$D031.5: ext_attrib_flag = %d\n", ext_attrib_flag);
  printf("\n");

  char* str = &outbuf[4]; // read the remainder of the string
  int x, y;
  int ret = parse_indices(str, &x, &y);
  if (!str || !ret)
  {
    printf("ERROR: Could not parse screen array indices!\n");
    return;
  }

  if (ret == 1) {
    check_if_setting_field(addr, chars_per_row, clr_base, str, x, y);
    print_seam(addr, chars_per_row, mcm_flag, ext_attrib_flag, clr_base, x, y);
  }

  if (ret == 2)
  {
    for (x = 0; x < chars_per_row; x++)
    {
      print_seam(addr, chars_per_row, mcm_flag, ext_attrib_flag, clr_base, x, y);

      if (ctrlcflag)
        break;
    }
  }
}

static mem_data* mmem = NULL;

int get_mm_byte(int addr)
{
  if (mmem == NULL || addr < mmem[0].addr || addr >= mmem[15].addr + 0x10) {
    mmem = get_mem28array(addr);
  }
  int chunk = (addr - mmem[0].addr) / 16;
  int idx = (addr - mmem[0].addr) % 16;
  return mmem[chunk].b[idx];
}

int get_mm_word(int addr)
{
  return get_mm_byte(addr) + (get_mm_byte(addr+1) << 8);
}

char mapCmds[128][12] =
{
  // 0x00 - 0x0f
  "", "", "BANK", "FILTER", "PLAY", "TEMPO", "MOVSPR", "SPRITE", "SPRCOLOR", "RREG", "ENVELOPE", "SLEEP", "CATALOG", "DOPEN", "APPEND", "DCLOSE",
  // 0x10 - 0x1f
  "BSAVE", "BLOAD", "RECORD", "CONCAT", "DVERIFY", "DCLEAR", "SPRSAV", "COLLISION", "BEGIN", "BEND", "WINDOW", "BOOT", "FREAD#", "WPOKE", "FWRITE#", "DMA",
  // 0x20 - 0x2f
  " ", "EDMA", " ", "MEM", "OFF", "FAST", "SPEED", "TYPE", "BVERIFY", "ECTORY", "ERASE", "FIND", "CHANGE", "SET", "SCREEN", "POLYGON",
  // 0x30 - 0x3f
  "ELLIPSE", "VIEWPORT", "GCOPY", "PEN", "PALETTE", "DMODE", "DPAT", "FORMAT", "TURBO", "FOREGROUND", " ", "BACKGROUND", "BORDER", "HIGHLIGHT", "MOUSE", "RMOUSE",
  // 0x40 - 0x4f
  "DISK", "CURSOR", "RCURSOR", "LOADIFF", "SAVEIFF", "EDIT", "FONT", "FGOTO", "FGOSUB", "MOUNT", "FREEZER", "CHDIR", "DOT", "INFO", "BIT", "UNLOCK",
  // 0x50 - 0x5f
  "LOCK", "MKDIR", "<<", ">>", "VSYNC", "", "", "", "", "", "", "", "", "", "", "",
  // 0x60 - 0x6f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x70 - 0x7f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
};

char mapPetscii[256][20] =
{
  // 0x00 - 0x0f
  "{0x00}", "{alt-palette}", "{underline-on}", "{0x03}", "{default-palette}", "{white}", "{0x06}", "{bell}", "{0x08}", "{tab}", "{linefeed}", "{disable-shift-mega}", "{enable-shift-mega}", "{return}", "{lower-case}", "{flash-on}",
  // 0x10 - 0x1f
  "{f9}", "{down}", "{reverse-on}", "{clrhome}", "{instdel}", "{f10}", "{f11}", "{f12}", "{toggle-tab}", "{f13}", "{f14}", "{escape}", "{red}", "{right}", "{green}", "{blue}",
  // 0x20 - 0x2f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x30 - 0x3f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x40 - 0x4f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x50 - 0x5f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x60 - 0x6f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x70 - 0x7f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",

  // 0x80 - 0x8f
  "{?}", "{orange}", "{underline-off}", "{shift-runstop}", "{help}", "{f1}", "{f3}", "{f5}", "{f7}", "{f2}", "{f4}", "{f6}", "{f8}", "{shift-return}", "{uppercase}", "{flash-off}",
  // 0x90 - 0x9f
  "{black}", "{up}", "{reverse-off}", "{shift-clrhome}", "{shift-instdel}", "{brown}", "{pink}", "{dk-grey}", "{grey}", "{lt-green}", "{lt-blue}", "{lt-grey}", "{purple}", "{left}", "{yellow}", "{cyan}",
  // 0xa0 - 0xaf
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0xb0 - 0xbf
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0xc0 - 0xcf
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0xd0 - 0xdf
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0xe0 - 0xef
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0xf0 - 0xff
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
};

char mapBasTok[256][12] =
{
  // 0x00 - 0x0f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x10 - 0x1f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x20 - 0x2f
  " ", "!", "\"", "#", "$", "%", "&", "'", "(", ")", "*", "+", ",", "-", ".", "/",
  // 0x30 - 0x3f
  "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", ":", ";", "<", "=", ">", "?",
  // 0x40 - 0x4f
  "", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O",
  // 0x50 - 0x5f
  "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "[", "{pound}", "]", "{upchr}", "{leftchr}",
  // 0x60 - 0x6f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x70 - 0x7f
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  // 0x80 - 0x8f
  "END", "FOR", "NEXT", "DATA", "INPUT#", "INPUT", "DIM", "READ", "LET", "GOTO", "RUN", "IF", "RESTORE", "GOSUB", "RETURN", "REM",
  // 0x90 - 0x9f
  "STOP", "ON", "WAIT", "LOAD", "SAVE", "VERIFY", "DEF", "POKE", "PRINT#", "PRINT", "CONT", "LIST", "CLR", "CMD", "SYS", "OPEN",
  // 0xa0 - 0xaf
  "CLOSE", "GET", "NEW", "TAB(", "TO", "FN", "SPC(", "THEN", "NOT", "STEP", "+", "-", "*", "/", "^", "AND",
  // 0xb0 - 0xbf
  "OR", ">", "=", "<", "SGN", "INT", "ABS", "USR", "FRE", "POS", "SQR", "RND", "LOG", "EXP", "COS", "SIN",
  // 0xc0 - 0xcf
  "TAN", "ATN", "PEEK", "LEN", "STR$", "VAL", "ASC", "CHR$", "LEFT$", "RIGHT$", "MID$", "GO", "RGRAPHIC", "RCOLOR", "", "JOY",
  // 0xd0 - 0xdf
  "RPEN", "DEC", "HEX$", "ERR$", "INSTR", "ELSE", "RESUME", "TRAP", "TRON", "TROFF", "SOUND", "VOL", "AUTO", "IMPORT", "GRAPHIC", "PAINT",
  // 0xe0 - 0xef
  "CHAR", "BOX", "CIRCLE", "PASTE", "CUT", "LINE", "MERGE", "COLOR", "SCNCLR", "XOR", "HELP", "DO", "LOOP", "EXIT", "DIR", "DSAVE",
  // 0xf0 - 0xff
  "DLOAD", "HEADER", "SCRATCH", "COLLECT", "COPY", "RENAME", "BACKUP", "DELETE", "RENUMBER", "KEY", "MONITOR", "USING", "UNTIL", "WHILE", "", "",
};

void cmdBasicList(void)
{
  char line[512] = { 0 };
  char s[50];
  mmem = NULL;

  int ptr = 0x2001;
  bool quote_flag = false;

  while (ptr != 0x0000)
  {
    int nextptr = get_mm_word(ptr);
    if (nextptr == 0x0000)
      break;

    ptr += 2;
    int linenum = get_mm_word(ptr);
    ptr += 2;
    sprintf(line, "%d ", linenum);

    int token;
    do
    {
      token = get_mm_byte(ptr);
      ptr++;
      if (token == 0xfe) // extended command?
      {
        token = get_mm_byte(ptr);
        ptr++;
        if (token >= 0x80)
        {
          sprintf(s, "{0x%02X}", token);
          strcat(line, s);
        }
        else
          strcat(line, mapCmds[token]);
      }
      else
      {
        if (token == '"')
          quote_flag = !quote_flag;

        if ( (mapBasTok[token][0] == '\0' && token != 0)
            || (quote_flag && (token < 0x20 || token >= 0x80) ) )
        {
          if (quote_flag)
          {
            strcat(line, mapPetscii[token]);
          }
          else
          {
            sprintf(s, "{0x%02X}", token);
            strcat(line, s);
          }
        }
        else
          strcat(line, mapBasTok[token]);
      }
      // printf("ptr=%04X, token=%02X\n", ptr, token);
      // if (ptr > 0x3140) return;
    }
    while (token != 0x00);

    printf("%s\n", line);
    ptr = nextptr;
  }
}


void cmdSprite(void)
{
  char* strSprIdx = strtok(NULL, " ");

  if (strSprIdx == NULL)
  {
    printf("Missing <spridx> parameter!\n");
    return;
  }

  int spridx = get_sym_value(strSprIdx);

  // is it an absolute address?
  if (strSprIdx[0] == '$') {
    spridx = get_sym_value(strSprIdx+1);
  }

  int vic_16kb_bank = mpeek(0xffd3d00) & 0x03;
  int vic_addr = (3 - vic_16kb_bank) * 0x4000;
  int spr_addr = vic_addr + spridx * 64;

  if (strSprIdx[0] == '$') {
    spr_addr = spridx;
  }

  // get memory at current pc
  mem_data* multimem = get_mem28array(spr_addr);
  int idx_cnt = 0;

  printf("vicii bank: $%04X - $%04X\n", vic_addr, vic_addr + 0x4000 - 1);
  printf("sprite idx $%02X address: $%04X\n", spridx, spr_addr);

  printf("+------------------------+\n|");
    for (int line = 0; line < 16; line++)
    {
      mem_data* mem = &multimem[line];

      for (int k = 0; k < 16; k++)
      {
        int val = mem->b[k];

        for (int bitfield = 128; bitfield >= 1; bitfield /= 2) {
          if ( (val & bitfield) != 0) {
            printf("*");
          } else {
            printf(" ");
          }
        }

        idx_cnt++;
        if ( (idx_cnt % 3) == 0) {
          printf("|\n");
          if (idx_cnt != 63) {
            printf("|");
          }
        }
        if (idx_cnt == 63) {
          printf("+------------------------+\n");
          return;
        }
      }
    }
}

void cmdSet(void)
{
  int addr;
  char command_str[1024] = { 0 };
  char byte_str[8] = { 0 };

  char* strAddr = strtok(NULL, " ");
  if (strAddr == NULL) {
    printf("Missing <addr> parameter!\n");
  }

  addr = get_sym_value(strAddr);

  sprintf(command_str, "s %04X ", addr);

  char* strValues = strtok(NULL, "\0");
  char* strVend = strValues + strlen(strValues);

  while (strValues[0] != '\0')
  {
    // provided a string
    if (strValues[0] == '\"')
    {
      strValues++; // skip the starting dbl quote
      while (strValues[0] != '\"' && strValues[0] != '\0') {
        sprintf(byte_str, "%02X ", strValues[0]);
        strcat(command_str, byte_str);
        if (strValues[0] != '\0')
          strValues++;
      }
      strValues++; // skip the ending dbl quote
    }
    else if (strValues[0] == ' '
        || strValues[0] ==',') { // skip spaces or commas
      strValues++;
    }
    else { // assume it is a hex byte
      char *sval = strtok(strValues, " ");
      int v = get_sym_value(sval);
      char nval[10];
      sprintf(nval, "%02X", v);
      strcat(command_str, nval);
      strcat(command_str, " ");
      strValues += strlen(sval);
      if (strValues != strVend && strValues[0] == '\0') // skip any \0 from strtok
        strValues++;
    }
  }
  strcat(command_str, "\n");
  printf("RAW: %s", command_str);

  serialWrite(command_str);
  serialRead(inbuf, BUFSIZE);
}


void cmdGo(void)
{
  int addr;
  char command_str[1024] = { 0 };

  char* strAddr = strtok(NULL, " ");
  if (strAddr == NULL) {
    printf("Missing <addr> parameter!\n");
  }

  addr = get_sym_value(strAddr);

  sprintf(command_str, "g %04X\n", addr);

  printf("RAW: %s", command_str);

  serialWrite(command_str);
  serialRead(inbuf, BUFSIZE);
}

void get_primary_colour_values(int address, unsigned char* paldata)
{
  mem_data* mem = get_mem28array(address);
  for (int k = 0; k < 256; k++)
  {
    int tmp = mem[k/16].b[k%16];
    paldata[k] = ((tmp & 0x0f) << 4) + (tmp >> 4);
  }
}

unsigned char* get_palette_data()
{
  static unsigned char paldata[3*256];

  get_primary_colour_values(0xffd3100, &paldata[0*256]);  // red
  get_primary_colour_values(0xffd3200, &paldata[1*256]);  // green
  get_primary_colour_values(0xffd3300, &paldata[2*256]);  // blue

  return paldata;
}

void set_palette_entry(int idx, int val)
{
  int r = (val >> 16) & 0xff;
  int g = (val >> 8) & 0xff;
  int b = val & 0xff;

  char str[128];
  int tmp = ( (r & 0x0f) << 4) + ( (r & 0xf0) >> 4);
  sprintf(str, "s%X %x\n", 0xffd3100 + idx, tmp);
  serialWrite(str);
  serialRead(inbuf, BUFSIZE);

  tmp = ( (g & 0x0f) << 4) + ( (g & 0xf0) >> 4);
  sprintf(str, "s%X %x\n", 0xffd3200 + idx, tmp);
  serialWrite(str);
  serialRead(inbuf, BUFSIZE);

  tmp = ( (b & 0x0f) << 4) + ( (b & 0xf0) >> 4);
  sprintf(str, "s%X %x\n", 0xffd3300 + idx, tmp);
  serialWrite(str);
  serialRead(inbuf, BUFSIZE);
}

void cmdPalette(void)
{
  int startidx = 0;
  int endidx = 31;

  char* strAddr = strtok(NULL, " ");
  if (strAddr != NULL) {
    startidx = get_sym_value(strAddr);
    endidx = startidx;

    strAddr = strtok(NULL, " ");
    if (strAddr != NULL) {
      if (strcmp(strAddr, "=") == 0) {
        strAddr = strtok(NULL, " ");
        if (strAddr == NULL) {
          printf("ERROR: Expected hex-value to set palette entry to\n");
          return;
        }
        int val = get_sym_value(strAddr);
        set_palette_entry(startidx, val);
      }
      else {
        endidx = get_sym_value(strAddr);
      }
    }
  }

  unsigned char *palette_mem = get_palette_data();

  for (int k = startidx; k <= endidx; k++)
  {
    unsigned int r = palette_mem[0*256 + k];
    unsigned int g = palette_mem[1*256 + k];
    unsigned int b = palette_mem[2*256 + k];
    printf("idx# 0x%02X: " KINV "\x1b[38;2;%u;%u;%um " KNRM " %02X%02X%02X (%u, %u, %u)\n", k, r, g, b, r, g, b, r, g, b);
  }
}

unsigned char* get_palette(void)
{
  static unsigned char paldata[256*3];

  unsigned char *palette_mem = get_palette_data();

  for (int k = 0; k <= 255; k++)
  {
    unsigned int r = palette_mem[0*256 + k];
    unsigned int g = palette_mem[1*256 + k];
    unsigned int b = palette_mem[2*256 + k];

    paldata[k*3 + 0] = r;
    paldata[k*3 + 1] = g;
    paldata[k*3 + 2] = b;
  }

  return paldata;
}


void clearListsAndMaps(void)
{
  // clear list data
  type_ci_chunk_info* iterCi = lstCalypsiChunkInfo;
  type_ci_chunk_info* curCi;

  while (iterCi != NULL) {
    curCi = iterCi;
    iterCi = iterCi->next;

    free(curCi->chunk_name);
    free(curCi->section);
    free(curCi);
  }

  lstCalypsiChunkInfo = NULL;

  type_fileloc* iterF = lstFileLoc;
  type_fileloc* curFileLoc;

  while (iterF != NULL) {
    curFileLoc = iterF;
    iterF = iterF->next;

    free(curFileLoc->file);
    free(curFileLoc);
  }

  lstFileLoc = NULL;

  // clear map data
  type_symmap_entry* iterS = lstSymMap;
  type_symmap_entry* curSym = NULL;

  while (iterS != NULL) {
    curSym = iterS;
    iterS = iterS->next;

    free(curSym->sval);
    free(curSym->symbol);
    free(curSym);
  }

  lstSymMap = NULL;
}


void cmdReload(void)
{
  clearListsAndMaps();
  listSearch();
}


void cmdChar(void)
{
  char* strCharIdx = strtok(NULL, " ");

  if (strCharIdx == NULL)
  {
    printf("Missing <charidx> parameter!\n");
    return;
  }

  int chridx = get_sym_value(strCharIdx);

  // is it an absolute address?
  if (strCharIdx[0] == '$') {
    chridx = get_sym_value(strCharIdx+1);
  }

  int cnt = 1;
  char* strCount = strtok(NULL, " ");
  if (strCount != NULL)
  {
    cnt = get_sym_value(strCount);
  }

  int inc = 1;
  char* strIncrement = strtok(NULL, " ");
  if (strIncrement != NULL)
  {
    inc = get_sym_value(strIncrement);
    if (inc == 0) {
      printf("increment value cannot be zero!\n");
      return;
    }
  }

  int vic_16kb_bank = mpeek(0xffd3d00) & 0x03;
  int vic_addr = (3 - vic_16kb_bank) * 0x4000;
  int chr_data_base_addr_selector = (mpeek(0xffd3018) >> 1) & 0x07;
  int chr_data_base_addr = vic_addr + 0x800 * chr_data_base_addr_selector;
  int chr_addr = chr_data_base_addr + chridx * 8;

  if (strCharIdx[0] == '$') {
    chr_addr = chridx;
  }

  printf("vicii bank: $%04X - $%04X\n", vic_addr, vic_addr + 0x4000 - 1);
  printf("char idx $%02X address: $%04X\n", chridx, chr_addr);

  for (int k = 0 ; k < cnt; k++)
    printf("+--------");
  printf("+\n");

  mem_data mem[cnt];

  for (int k = 0; k < cnt; k++)
  {
    mem[k] = get_mem(chr_addr + (k*inc) * 8, false);
  }

  for (int k = 0; k < 8; k++)
  {
    for (int ck = 0; ck < cnt; ck++)
    {
      int val = mem[ck].b[k];

      printf("|");
      for (int bitfield = 128; bitfield >= 1; bitfield /= 2) {
        if ( (val & bitfield) != 0) {
          printf("*");
        } else {
          printf(" ");
        }
      }
    }
    printf("|\n");
  }

  for (int k = 0; k < cnt; k++)
    printf("+--------");
  printf("+\n");
}

poke_bitfield_info find_bitfield(char* str)
{
  int start_bit, end_bit, matched_chars;
  poke_bitfield_info bfi;
  bfi.found = false;

  char* f = strchr(str, '.');
  if (f == NULL)
    return bfi;

  *f = '\0';
  f++;
  if (sscanf(f, "%d-%d %n", &start_bit, &end_bit, &matched_chars) == 2) {
    if (start_bit > end_bit) {
      int swap = start_bit;
      start_bit = end_bit;
      end_bit = swap;
    }

    bfi.found = true;
    bfi.start_bit = start_bit;
    bfi.num_bits = end_bit - start_bit + 1;
  }
  else if (sscanf(f, "%d %n", &start_bit, &matched_chars) == 1) {
    bfi.found = true;
    bfi.start_bit = start_bit;
    bfi.num_bits = 1;
  }

  return bfi;
}

void cmd_poke(int size, bool is_addr28)
{
  char* strAddr = strtok(NULL, " ");

  if (strAddr == NULL)
  {
    printf("Missing <addr> parameter!\n");
    return;
  }

  poke_bitfield_info bfi = {0};
  if (size == 1) {  // bitfield support is only available for poke & mpoke (not pokew/poked/pokeq)
    bfi = find_bitfield(strAddr);
  }

  int addr28 = get_sym_value(strAddr);

  if (!is_addr28)
    addr28 += 0x7770000;

  sprintf(outbuf, "s%08X", addr28);

  char str[10] = "";
  char * strVal;
  unsigned long val;

  if (bfi.found) {  // are we updating just a bitfield section?
    if ( (strVal = strtok(NULL, " ")) != NULL) {
      val = get_sym_value(strVal);

      int memval;
      int oldmemval;
      oldmemval = mpeek(addr28);
      memval = oldmemval;
      set_field(&memval, bfi.start_bit, bfi.num_bits, val);

      sprintf(str, " %X", memval & 0xff);
      strcat(outbuf, str);
      printf("\x1b[38;2;255;0;0mBEFORE: $%02X : %%%s\n", oldmemval, toBinaryString(oldmemval, &bfi));
      printf("\x1b[38;2;0;255;0m AFTER: $%02X : %%%s%s\n", memval, toBinaryString(memval, &bfi), KNRM);
    }
  }
  else {  // we poke to a normal address, byte per byte
    while ( (strVal = strtok(NULL, " ")) != NULL)
    {
      val = get_sym_value(strVal);

      for (int k = 0; k < size; k++)
      {
        sprintf(str, " %lX", val & 0xff);
        strcat(outbuf, str);
        val >>= 8;
      }
    }
  }

  serialWrite(outbuf);
  serialRead(inbuf, BUFSIZE);
}

void cmdPoke(void)
{
  cmd_poke(1, false);
}
void cmdPokeW(void)
{
  cmd_poke(2, false);
}
void cmdPokeD(void)
{
  cmd_poke(4, false);
}
void cmdPokeQ(void)
{
  cmd_poke(8, false);
}


void cmdMPoke(void)
{
  cmd_poke(1, true);
}

void cmdMPokeW(void)
{
  cmd_poke(2, true);
}
void cmdMPokeD(void)
{
  cmd_poke(4, true);
}
void cmdMPokeQ(void)
{
  cmd_poke(8, true);
}

// write buffer to client ram
void put_mem28array(int addr, unsigned char* data, int size)
{
  char str[10];
  sprintf(outbuf, "s%08X", addr);

  int i = 0;
  while(i < size)
  {
    sprintf(str, " %02X", data[i]);
    strcat(outbuf, str);
    i++;
  }
  strcat(outbuf, "\n");

  serialWrite(outbuf);
  serialRead(inbuf, BUFSIZE);
}

void cmdRawHelp(void)
{
  serialWrite("?\n");
  serialRead(inbuf, BUFSIZE);
  printf("%s", inbuf);

  printf("! - reset machine\n"
         "f<low> <high> <byte> - Fill memory\n"
         "g<addr> - Set PC\n"
         "m<addr28> - Dump 16-bytes of memory (28bit addresses - use $777xxxx for CPU context)\n"
         "M<addr28> - Dump 512-bytes of memory (28bit addresses - use $777xxxx for CPU context)\n"
         "d<addr> - Disassemble one instruction (28bit addresses - use $777xxxx for CPU context)\n"
         "D<addr> - Disassemble 16 instructions (28bit addresses - use $777xxxx for CPU context)\n"
         "r - display CPU registers and last instruction executed\n"
         "s<addr28> <value> ... - Set memory (28bit addresses)\n"
         "S<addr> <value> ... - Set memory (CPU memory context)\n"
         "b[<addr>] - Set or clear CPU breakpoint\n"
         "t<0|1> - Enable/disable tracing\n"
         "tc - Traced execution until keypress\n"
         "t|BLANK LINE - Step one cpu cycle if in trace mode\n"
         "w<addr> - Sets a watchpoint to trigger when specified address is modified\n"
         "w - clear 'w' watchpoint\n"
         "e - set a breakpoint to occur based on CPU flags\n"
         "z - history command - show every memory access on the bus for the last hundred cycles\n"
         "+ - set bitrate (+9 = 4,000,000bps,  +13 = 2,000,000bps)\n"
         "i - irq command - (for masking interrupts)\n"
         "# - trap command - trigger a trap?\n"
         "E - flag break command - allows breaking on particular CPU flag settings\n"
         "l<addr28> <addr16> - load mem command (28-bit start address, lower 16-bits of end address, then feed in raw bytes)\n"
         "N - step over (xemu only)\n"
  );
}

char *strlower(char *str)
{
  unsigned char *p = (unsigned char *)str;

  while (*p) {
     *p = tolower((unsigned char)*p);
      p++;
  }

  return str;
}

void cmdHelp(void)
{
  // get address from parameter?
  char* cmdname = strtok(NULL, " ");
  if (cmdname)
    strlower(cmdname);

  if (cmdname == NULL) {
    printf("m65dbg commands\n"
           "===============\n");
  }

  for (int k = 0; command_details[k].name != NULL; k++)
  {
    type_command_details cd = command_details[k];

    if (cmdname == NULL || strcmp(cd.name, cmdname) == 0)
    {
      if (cd.params == NULL)
        printf("%s = %s\n", cd.name, cd.help);
      else
        printf("%s %s = %s\n", cd.name, cd.params, cd.help);
    }
  }

  if (cmdname == NULL) {
    printf(
     "[ENTER] = repeat last command\n"
     "q/x/exit = exit the program\n"
     );
  }
}

void print_char(int c)
{
  if (petscii)
  {
    print_screencode(c, 1);
  }
  else
  {
    if (c >= 192 && c <= 223)   // codes from 192-223 are equal to 96-127
      c -= 96;

    if (isprint(c) && c > 31) {

      printf("%c", c);
    }
    else
      printf(".");
  }
}

void dump(int addr, int total)
{
  int cnt = 0;
  while (cnt < total)
  {
    // get memory at current pc
    mem_data mem = get_mem(addr + cnt, false);

    printf(" :%07X ", mem.addr);
    for (int k = 0; k < 16; k++)
    {
      if (k == 8) // add extra space prior to 8th byte
        printf(" ");

      printf("%02X ", mem.b[k]);
    }

    printf(" | ");

    for (int k = 0; k < 16; k++)
    {
      int c = mem.b[k];
      print_char(c);
    }
    printf("\n");
    cnt+=16;

    if (ctrlcflag)
      break;
  }
}

void cmdDump(void)
{
  char* strAddr = strtok(NULL, " ");

  if (strAddr == NULL)
  {
    printf("Missing <addr> parameter!\n");
    return;
  }

  int addr = get_sym_value(strAddr);

  int total = 16;
  char* strTotal = strtok(NULL, " ");

  if (strTotal != NULL)
  {
    sscanf(strTotal, "%X", &total);
  }

  dump(addr, total);
}

void mdump(int addr, int total)
{
  int cnt = 0;
  while (cnt < total)
  {
    // get memory at current pc
    mem_data mem = get_mem(addr + cnt, true);

    printf(" :%07X ", mem.addr);
    for (int k = 0; k < 16; k++)
    {
      if (k == 8) // add extra space prior to 8th byte
        printf(" ");

      if (cnt+k >= total) {
        printf("   ");
        continue;
      }

      printf("%02X ", mem.b[k]);
    }

    printf(" | ");

    for (int k = 0; k < 16; k++)
    {
      if (cnt+k == total)
        break;

      int c = mem.b[k];
      print_char(c);
    }
    printf("\n");
    cnt+=16;

    if (ctrlcflag)
      break;
  }
}

void cmdMDump(void)
{
  char* strAddr = strtok(NULL, " ");

  if (strAddr == NULL)
  {
    printf("Missing <addr> parameter!\n");
    return;
  }

  int addr = get_sym_value(strAddr);

  int total = 16;
  char* strTotal = strtok(NULL, " ");

  if (strTotal != NULL)
  {
    sscanf(strTotal, "%X", &total);
  }

  mdump(addr, total);
}

// return the last byte count
int disassemble_addr_into_string(char* str, int addr, bool useAddr28)
{
  int last_bytecount = 0;
  char s[32] = { 0 };

  // get memory at current pc
  mem_data mem = get_mem(addr, useAddr28);

  // now, try to disassemble it

  // Program counter
  if (useAddr28)
    sprintf(str, "$%07X ", addr & 0xfffffff);
  else
    sprintf(str, "$%04X ", addr & 0xffff);

  type_opcode_mode mode = opcode_mode[mode_lut[mem.b[0]]];
  sprintf(s, " %10s:%d ", mode.name, mode.val);
  strcat(str, s);

  // Opcode and arguments
  sprintf(s, "%02X ", mem.b[0]);
  strcat(str, s);

  last_bytecount = mode.val + 1;

  if (last_bytecount == 1)
  {
    strcat(str, "      ");
  }
  if (last_bytecount == 2)
  {
    sprintf(s, "%02X    ", mem.b[1]);
    strcat(str, s);
  }
  if (last_bytecount == 3)
  {
    sprintf(s, "%02X %02X ", mem.b[1], mem.b[2]);
    strcat(str, s);
  }

  // Instruction name
  sprintf(s, "%-4s", instruction_lut[mem.b[0]]);
  strcat(str, s);

  switch(mode_lut[mem.b[0]])
  {
    case M_impl: break;
    case M_InnX:
      sprintf(s, " ($%02X,X)", mem.b[1]);
      strcat(str, s);
      break;
    case M_nn:
      sprintf(s, " $%02X", mem.b[1]);
      strcat(str, s);
      break;
    case M_immnn:
      sprintf(s, " #$%02X", mem.b[1]);
      strcat(str, s);
      break;
    case M_A: break;
    case M_nnnn:
      sprintf(s, " $%02X%02X", mem.b[2], mem.b[1]);
      strcat(str, s);
      break;
    case M_nnrr:
      sprintf(s, " $%02X,$%04X", mem.b[1], (addr + 3 + mem.b[2]) );
      strcat(str, s);
      break;
    case M_rr:
      if (mem.b[1] & 0x80)
        sprintf(s, " $%04X", (addr + 2 - 256 + mem.b[1]) );
      else
        sprintf(s, " $%04X", (addr + 2 + mem.b[1]) );
      strcat(str, s);
      break;
    case M_InnY:
      sprintf(s, " ($%02X),Y", mem.b[1]);
      strcat(str, s);
      break;
    case M_InnZ:
      sprintf(s, " ($%02X),Z", mem.b[1]);
      strcat(str, s);
      break;
    case M_rrrr:
      sprintf(s, " $%04X", (addr + 2 + (mem.b[2] << 8) + mem.b[1]) & 0xffff );
      strcat(str, s);
      break;
    case M_nnX:
      sprintf(s, " $%02X,X", mem.b[1]);
      strcat(str, s);
      break;
    case M_nnnnY:
      sprintf(s, " $%02X%02X,Y", mem.b[2], mem.b[1]);
      strcat(str, s);
      break;
    case M_nnnnX:
      sprintf(s, " $%02X%02X,X", mem.b[2], mem.b[1]);
      strcat(str, s);
      break;
    case M_Innnn:
      sprintf(s, " ($%02X%02X)", mem.b[2], mem.b[1]);
      strcat(str, s);
      break;
    case M_InnnnX:
      sprintf(s, " ($%02X%02X,X)", mem.b[2], mem.b[1]);
      strcat(str, s);
      break;
    case M_InnSPY:
      sprintf(s, " ($%02X,SP),Y", mem.b[1]);
      strcat(str, s);
      break;
    case M_nnY:
      sprintf(s, " $%02X,Y", mem.b[1]);
      strcat(str, s);
      break;
    case M_immnnnn:
      sprintf(s, " #$%02X%02X", mem.b[2], mem.b[1]);
      strcat(str, s);
      break;
  }

  return last_bytecount;
}

int* get_backtrace_addresses(void)
{
  // get current register values
  reg_data reg = get_regs();

  static int addresses[8];

  // get memory at current pc
  mem_data mem = get_mem(reg.sp+1, false);
  for (int k = 0; k < 8; k++)
  {
    int addr = mem.b[k*2] + (mem.b[k*2+1] << 8);
    addr -= 2;
    addresses[k] = addr;
  }

  return addresses;
}

#define PCCNT 400

int zpos = 0;
int zval[PCCNT];
int zflag = 0;

void getz(void)
{
  serialWrite("z\n");
  serialRead(inbuf, BUFSIZE);

  zflag = 1;

  char* strLine = strtok(inbuf, "\n");
  for (int k = 0; k < PCCNT; k++)
  {
    sscanf(strLine, "%X", &zval[k]);
    strLine = strtok(NULL, "\n");
  }
}

void disassemble(bool useAddr28)
{
  char str[128] = { 0 };
  int last_bytecount = 0;

  if (autowatch)
    cmdWatches();

  if (autolocals) {
    cmdLocals();
    printf("---------------------------------------\n");
  }

  int addr;
  int cnt = 1; // number of lines to disassemble

  // get current register values
  reg_data reg = get_regs();
  if (autowatch)
  {
    show_regs(&reg);
    printf("---------------------------------------\n");
  }

  // get address from parameter?
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "-") == 0) // '-' equates to current pc
    {
      // get current register values
      addr = reg.pc;
    }
    else
      addr = get_sym_value(token);

    token = strtok(NULL, " ");

    if (token != NULL)
    {
      cnt = get_sym_value(token);
    }
  }
  // default to current pc
  else
  {
    addr = reg.pc;
  }

  // are we in a different frame?
  if (addr == reg.pc && traceframe != 0)
  {
    int* addresses = get_backtrace_addresses();
    addr = addresses[traceframe-1];

    printf("<<< FRAME#: %d >>>\n", traceframe);
  }

  int idx = 0;

  while (idx < cnt)
  {
    last_bytecount = disassemble_addr_into_string(str, addr, useAddr28);

    // print from .list ref? (i.e., find source in .a65 file?)
    if (idx == 0)
    {
      type_fileloc *found = find_in_list(addr);
      cur_file_loc = found;
      if (found)
      {
        if (found->module)
          printf("> \"%s\"  (%s:%d)\n", found->module, found->file, found->lineno);
        else
          printf("> %s::%d\n", found->file, found->lineno);
        show_location(found);
        printf("---------------------------------------\n");
      }
    }

    // just print the raw disassembly line
    if (cnt != 1 && idx == 0)
      printf("%s%s%s\n", KINV, str, KNRM);
    else
      printf("%s\n", str);

    if (ctrlcflag)
      break;

    addr += last_bytecount;
    idx++;
  } // end while
}

void cmdForwardDis(void)
{
  static char s[64];
  int cnt = 1;

  if (!zflag)
    getz();

  char* token = strtok(NULL, " ");
  if (token != NULL)
    sscanf(token, "%d", &cnt);

  if (zpos-cnt > 0)
    zpos-=cnt;
  else
    zpos=0;

  sprintf(s, "dis %04X", zval[zpos]);
  strtok(s, " ");

  disassemble(false);
}

void cmdBackwardDis(void)
{
  static char s[64];
  int cnt = 1;

  if (!zflag)
    getz();

  char* token = strtok(NULL, " ");
  if (token != NULL)
    sscanf(token, "%d", &cnt);

  if (zpos+cnt < PCCNT)
    zpos+=cnt;
  else
    zpos = PCCNT-1;

  sprintf(s, "dis %04X", zval[zpos]);
  strtok(s, " ");

  disassemble(false);
}

void cmdMCopy(void)
{
  // get address from parameter?
  char* token = strtok(NULL, " ");

  if (token == NULL)
  {
    printf("Invalid args\n");
    return;
  }

  int src_addr = get_sym_value(token);

  token = strtok(NULL, " ");

  if (token == NULL)
  {
    printf("Invalid args\n");
    return;
  }

  int dest_addr = get_sym_value(token);

  token = strtok(NULL, " ");

  if (token == NULL)
  {
    printf("Invalid args\n");
    return;
  }

  int count = get_sym_value(token);

  char strCmd[256];
  int cnt = 0;
  while (cnt < count)
  {
    mem_data mem = get_mem(src_addr + cnt, true);
    sprintf(strCmd, "s%X ", dest_addr + cnt);
    for (int k = 0; k < 16; k++)
    {
      char strVal[10];
      sprintf(strVal, "%02X ", mem.b[k]);
      strcat(strCmd, strVal);
      cnt++;
      if (cnt >= count)
        break;
    }
    printf("%d%%...\r", 100*cnt/count);
    strcat(strCmd,"\n");
    serialWrite(strCmd);
    serialRead(inbuf, BUFSIZE);

    if (ctrlcflag)
      break;
  }
  printf("\n");
}

type_funcinfo* find_current_function(int pc)
{
  type_funcinfo* previter = 0;
  type_funcinfo* iter = lstFuncInfo;

  while (iter != NULL) {
    // printf("%s : $%04X\n", iter->name, iter->addr);

    if (iter->addr > pc)
    {
      if (previter == 0 || (iter->addr - pc) < 8)
        return NULL;

      //printf("FOUND IT!\n");
      return previter;
    }

    previter = iter;
    iter = iter->next;
  }
  return NULL;
}

int get_sptop(type_localinfo* iter) {
  mem_data mem = get_mem(0x02, false);
  int sp = mem.b[0] + (mem.b[1] << 8);

  int* addresses = get_backtrace_addresses();
  if (traceframe != 0) {
    for (int k = traceframe; k >=0; k--) {
      int addr = addresses[k-1];
      type_funcinfo* fi = find_current_function(addr);
      if (fi != NULL) {
        sp += fi->paramsize;
      }
    }
  }

  while (iter != NULL) {
    if (iter->offset < 0) {
      sp += iter->size;
    }

    iter = iter->next;
  }
  return sp;
}

void cmdLocals(void)
{
  int addr;

  // get current pc
  reg_data reg = get_regs();
  addr = reg.pc;

  if (traceframe != 0) {
    int* addresses = get_backtrace_addresses();
    addr = addresses[traceframe-1];
  }

  // assess which function it resides in
  type_funcinfo* fi = find_current_function(addr);
  if (fi == NULL)
    return;

  // iterate over list of locals within function
  type_localinfo* iter = fi->locals;
  int sptop = get_sptop(iter);
  if (iter != NULL)
    printf("LOCALS: %s\n", fi->name);
  else
    printf("LOCALS: %s\nnone found!\n", fi->name);

  while (iter != NULL) {
    int addr = sptop + iter->offset;
    printf("@ $%04X :", addr);
    if (iter->size == 1)
      print_byte_at_addr(iter->name, addr, false, false, false, false);
    else if (iter->size == 2)
      print_word_at_address(iter->name, addr, false, false);
    else if (iter->size == 4)
      print_dword_at_address(iter->name, addr, false, false);
    else {
      printf(" %s[%d] =\n", iter->name, iter->size);
      dump(addr, iter->size);
    }

    // read (sptop-offset) locations of memory to print out local values

    iter = iter->next;
  }
}

void cmdDisassemble(void)
{
  disassemble(false);
}

void strupper(char* str)
{
  char* s = str;
  while (*s)
  {
    *s = toupper((unsigned char)*s);
    s++;
  }
}

void write_bytes(int* addr, int size, ...)
{
  va_list valist;
  va_start(valist, size);

  char str[16];
  sprintf(outbuf, "s%X", *addr);

  int i = 0;
  while(i < size)
  {
    sprintf(str, " %02X", va_arg(valist, int));
    strcat(outbuf, str);
    i++;
  }
  strcat(outbuf, "\n");

  serialWrite(outbuf);
  serialRead(inbuf, BUFSIZE);

  va_end(valist);

  *addr += size;
}

int isValidMnemonic(char* str)
{
  strupper(str);
  char* instr = strtok(str, " ");
  for (int k = 0; k < 256; k++)
  {
    if (strcmp(instruction_lut[k], instr) == 0)
      return 1;
  }
  return 0;
}

int getopcode(int mode, const char* instr)
{
  for (int k = 0; k < 256; k++)
  {
    if (strcmp(instruction_lut[k], instr) == 0)
    {
      if (mode_lut[k] == mode)
        return k;
    }
  }

  // couldn't find it
  return -1;
}

int modematcher(char* curmode, char* modeform, int* var1, int* var2)
{
  char str[8];
  char* p = curmode;
  int varcnt = 0;

  for (int k = 0; k < strlen(modeform); k++)
  {
    // skip any whitespaces in curmode
    while (*p == ' ')
      p++;

    // did we locate a var position?
    if (modeform[k] == '%')
    {
      k++; // skip over the '%x' token

      if (!isxdigit((int)*p))  // if no hexadecimal value here, then we've failed to match
        return 0;

      str[0] = '\0';
      int i = 0;
      while(isxdigit((int)*p)) // skip over the hexadecimal value
      {
        str[i] = *p;
        i++;
        p++;
      }
      str[i] = '\0';

      if (varcnt == 0)
        sscanf(str, "%x", var1);
      else
        sscanf(str, "%x", var2);
      varcnt++;
    }
    // check for non-var locations, such as '(', ')', ','
    else if (modeform[k] == *p)
    {
      p++;
    }
    else // we failed to match the non-var location
    {
      return 0;
    }
  }

  return varcnt;
}

// return -1=invalid-command, 0=no-command, 1=bytes written
int oneShotAssembly(int* paddr, char* strCommand)
{
  char str[128];
  int val1;
  int val2;
  int opcode = 0;
  int startaddr = *paddr;
  int invalid = 0;
  strcpy(str, strCommand);
  if (str[strlen(str)-1] == '\n')
    str[strlen(str)-1] = '\0';

  if (strlen(str) == 0)
    return 0;

  strupper(str);

  char* instr = strtok(str, " ");
  char* mode = strtok(NULL, "\0");

  // todo: check if instruction is valid.
  // if not, show syntax error.

  if (strcmp(instr, "NOP") == 0)
    strcpy(instr, "EOM");

  // figure out instruction mode
  if (mode == NULL || mode[0] == '\0')
  {
    if ((opcode = getopcode(M_impl, instr)) != -1)
    {
      write_bytes(paddr, 1, opcode);
    }
    else if ((opcode = getopcode(M_A, instr)) != -1)
    {
      write_bytes(paddr, 1, opcode);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "($%x,X)",&val1, &val2) == 1)
  {
    if ((opcode = getopcode(M_InnX, instr)) != -1 && val1 < 0x100)
    {
      // e.g. ORA ($20,X)
      write_bytes(paddr, 2, opcode, val1);
    }
    else if ((opcode = getopcode(M_InnnnX, instr)) != -1)
    {
      // e.g. JSR ($2000,X)
      write_bytes(paddr, 3, opcode, val1 & 0xff, val1 >> 8);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "$%x,$%x",&val1, &val2) == 2)
  {
    if ((opcode = getopcode(M_nnrr, instr)) != -1)
    {
      // e.g. BBR $20,$2005
      int rr = 0;
      // TODO: confirm this arithmetic (for M_nnrr in disassemble_addr_into_string() too)
      if (val2 > *paddr)
        rr = val2 - *paddr - 2;
      else
        rr = (val2 - *paddr - 2) & 0xff;

      write_bytes(paddr, 3, opcode, val1, rr);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "($%x),Y", &val1, &val2) == 1)
  {
    if ((opcode = getopcode(M_InnY, instr)) != -1)
    {
      // e.g. ORA ($20),Y
      write_bytes(paddr, 2, opcode, val1);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "($%x),Z", &val1, &val2) == 1)
  {
    if ((opcode = getopcode(M_InnZ, instr)) != -1)
    {
      // e.g. ORA ($20),Z
      write_bytes(paddr, 2, opcode, val1);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "$%x,X", &val1, &val2) == 1)
  {
    if ((opcode = getopcode(M_nnX, instr)) != -1 && val1 < 0x100)
    {
      // e.g. ORA $20,X
      write_bytes(paddr, 2, opcode, val1);
    }
    else if ((opcode = getopcode(M_nnnnX, instr)) != -1)
    {
      // e.g. ORA $2000,X
      write_bytes(paddr, 3, opcode, val1 & 0xff, val1 >> 8);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "$%x,Y", &val1, &val2) == 1)
  {
    if ((opcode = getopcode(M_nnY, instr)) != -1 && val1 < 0x100)
    {
      // e.g. STX $20,Y
      write_bytes(paddr, 2, opcode, val1);
    }
    else if ((opcode = getopcode(M_nnnnY, instr)) != -1)
    {
      // e.g. ORA $2000,Y
      write_bytes(paddr, 3, opcode, val1 & 0xff, val1 >> 8);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "($%x)", &val1, &val2) == 1)
  {
    if ((opcode = getopcode(M_Innnn, instr)) != -1)
    {
      // e.g. JSR ($2000)
      write_bytes(paddr, 3, opcode, val1 & 0xff, val1 >> 8);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "($%x,SP),Y", &val1, &val2) == 1)
  {
    if ((opcode = getopcode(M_InnSPY, instr)) != -1)
    {
      // e.g. STA ($20,SP),Y
      write_bytes(paddr, 2, opcode, val1);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "$%x",&val1, &val2) == 1)
  {
    int diff = val1 - *paddr;
    if (diff < 0) diff = -diff;

    if ((opcode = getopcode(M_nn, instr)) != -1 && val1 < 0x100)
    {
      // e.g. STA $20
      write_bytes(paddr, 2, opcode, val1);
    }
    else if ((opcode = getopcode(M_nnnn, instr)) != -1)
    {
      // e.g. STA $2000
      write_bytes(paddr, 3, opcode, val1 & 0xff, val1 >> 8);
    }

    else if ((opcode = getopcode(M_rr, instr)) != -1 && diff < 0x100)
    {
      int rr = 0;
      // TODO: confirm this arithmetic (for M_rr in disassemble_addr_into_string() too)
      if (val1 > *paddr)
        rr = val1 - *paddr - 2;
      else
        rr = (val1 - *paddr - 2) & 0xff;
      // e.g. BCC $2005
      write_bytes(paddr, 2, opcode, rr);
    }
    else if ((opcode = getopcode(M_rrrr, instr)) != -1)
    {
      int rrrr = 0;
      // TODO: confirm this arithmetic (for M_rrrr in disassemble_addr_into_string() too)
      if (val1 > *paddr)
        rrrr = val1 - *paddr - 2;
      else
        rrrr = (val1 - *paddr - 2) & 0xffff;
      // e.g. BPL $AD22
      write_bytes(paddr, 3, opcode, rrrr & 0xff, rrrr >> 8);
    }
    else
    {
      invalid=1;
    }
  }
  else if (modematcher(mode, "#$%x",&val1, &val2) == 1)
  {
    // M_immnn
    // M_immnnnn
    if ((opcode = getopcode(M_immnn, instr)) != -1 && val1 < 0x100)
    {
      // e.g. LDA #$20
      write_bytes(paddr, 2, opcode, val1);
    }
    else if ((opcode = getopcode(M_immnnnn, instr)) != -1)
    {
      // e.g. PHW #$1234
      write_bytes(paddr, 3, opcode, val1 & 0xff, val1 >> 8);
    }
    else
    {
      invalid=1;
    }
  }
  else
  {
    invalid = 1;
  }

  if (invalid)
    return -1;

  // return number of bytes written
  return *paddr - startaddr;
}


void cmdAssemble(void)
{
  int addr;
  char str[128] = { 0 };
  char* token = strtok(NULL, " ");
  if (token != NULL)
  {
    addr = get_sym_value(token);
  }

  do
  {
    printf("$%07X ", addr);
    fgets(str, 128, stdin);
    int ret = oneShotAssembly(&addr, str);

    if (ret == -1)  // invalid?
    {
      printf("???\n");
    }
    else if (ret == 0) // no command?
    {
      return;
    }

  } while (1);
}

void cmdMDisassemble(void)
{
  disassemble(true);
}


void do_continue(int do_soft_break)
{
  traceframe = 0;

  // get address from parameter?
  char* token = strtok(NULL, " ");

  // if <addr> field is provided, use it
  if (token)
  {
    int addr = get_sym_value(token);

    if (do_soft_break) {
    // set a soft breakpoint (more reliable for now...)
    // doOneShotAssembly("sei");
    step(); // step just once, just in-case user wants to repeat the last continue command (e.g., 'c 3aa9')
    setSoftBreakpoint(addr);
    }
    else
    {
      // set a hard breakpoint
      char str[100];
      sprintf(str, "b%04X\n", addr);
      serialWrite(str);
      serialRead(inbuf, BUFSIZE);
    }

  }

  // just send an enter command
  serialWrite("t0\n");
  serialRead(inbuf, BUFSIZE);

  // Try keep this in a loop that tests for a breakpoint
  // getting hit, or the user pressing CTRL-C to force
  // a "t1" command to turn trace mode back on
  int cur_pc = -1;
  int same_cnt = 0;
  continue_mode = true;
  while ( 1 )
  {
    usleep(10000);

    if (ctrlcflag) {
      break;
    }

    // get current register values
    reg_data reg = get_regs();

    if (reg.pc == cur_pc ||
        (softbrkaddr && (reg.pc >= (softbrkaddr & 0xffff) && reg.pc <= ((softbrkaddr+3) & 0xffff))))
    {
      same_cnt++;
      // printf("good addr=$%04X : same_cnt=%d (soft=$%04X)\n", reg.pc, same_cnt, softbrkaddr);
      if (same_cnt == 5)
      {
        if (reg.pc >= (softbrkaddr & 0xffff) && reg.pc <= ((softbrkaddr + 3) & 0xffff)) {
          clearSoftBreak();
          // doOneShotAssembly("cli");
        }
        break;
      }
    }
    else
    {
      // printf("lost addr=$%04X (soft=$%04X)\n", reg.pc, softbrkaddr);
      same_cnt = 0;
      cur_pc = reg.pc;
    }
  }

  if (ctrlcflag)
  {
    serialWrite("t1\n");
    serialRead(inbuf, BUFSIZE);
  }

  continue_mode = false;
  if (autocls)
    cmdClearScreen();

  // show the registers
  // serialWrite("r\n");
  // serialRead(inbuf, BUFSIZE);
  // printf("%s", inbuf);

  cmdDisassemble();
}

void cmdSoftContinue(void)
{
  do_continue(1);
}

void cmdContinue(void)
{
  do_continue(0);
}


bool cmdGetContinueMode(void)
{
  return continue_mode;
}

void cmdSetContinueMode(bool val)
{
  continue_mode = val;
}

void hard_next(void)
{
  serialWrite("N\n");
  serialRead(inbuf, BUFSIZE);
}

void step(void)
{
  // just send an enter command
  serialWrite("\n");
  serialRead(inbuf, BUFSIZE);
}

void cmdHardNext(void)
{
  traceframe = 0;

  // get address from parameter?
  char* token = strtok(NULL, " ");

  int count = 1;

  // if <count> field is provided, use it
  if (token)
  {
    sscanf(token, "%d", &count);
  }

  // get current register values
  reg_data reg = get_regs();

  for (int k = 0; k < count; k++)
  {
    type_fileloc *found = find_in_list(reg.pc);
    cur_file_loc = found;

    if (found != NULL && found->lastaddr != 0 && (found->addr <= reg.pc && reg.pc <= found->lastaddr))
    {
      do
      {
        hard_next();
        reg = get_regs();
      } while(reg.pc <= found->lastaddr);
    }
    else
    {
      hard_next();
    }
  }

  if (outputFlag)
  {
    if (autocls)
      cmdClearScreen();
    printf("%s", inbuf);
    cmdDisassemble();
  }
}

void cmdStep(void)
{
  traceframe = 0;

  // get address from parameter?
  char* token = strtok(NULL, " ");

  int count = 1;

  // if <count> field is provided, use it
  if (token)
  {
    sscanf(token, "%d", &count);
  }

  for (int k = 0; k < count; k++)
  {
    step();
  }

  if (outputFlag)
  {
    if (autocls)
      cmdClearScreen();
    printf("%s", inbuf);
    cmdDisassemble();
  }
}

void cmdNext(void)
{
  traceframe = 0;

  // get address from parameter?
  char* token = strtok(NULL, " ");

  int count = 1;

  // if <count> field is provided, use it
  if (token)
  {
    sscanf(token, "%d", &count);
  }

  for (int k = 0; k < count; k++)
  {
    // check if this is a JSR command
    reg_data reg = get_regs();
    mem_data mem = get_mem(reg.pc, false);

    // if not, then just do a normal step
    if (strcmp(instruction_lut[mem.b[0]], "JSR") != 0)
    {
      step();
    }
    else
    {
      // if it is JSR, then keep doing step into until it returns to the next command after the JSR

      type_opcode_mode mode = opcode_mode[mode_lut[mem.b[0]]];
      int last_bytecount = mode.val + 1;
      int next_addr = reg.pc + last_bytecount;

      while (reg.pc != next_addr)
      {
        // just send an enter command
        serialWrite("\n");
        serialRead(inbuf, BUFSIZE);

        reg = get_regs();

        if (ctrlcflag)
          break;
      }

      // show disassembly of current position
      serialWrite("r\n");
      serialRead(inbuf, BUFSIZE);
    } // end if
  } // end for

  if (outputFlag)
  {
    if (autocls)
      cmdClearScreen();
    // printf("%s", inbuf);
    cmdDisassemble();
  }
}


void cmdFinish(void)
{
  traceframe = 0;

  reg_data reg = get_regs();

  int cur_sp = reg.sp;
  bool function_returning = false;

  //outputFlag = false;
  while (!function_returning)
  {
    reg = get_regs();
    mem_data mem = get_mem(reg.pc, false);

    if ((strcmp(instruction_lut[mem.b[0]], "RTS") == 0 ||
                     strcmp(instruction_lut[mem.b[0]], "RTI") == 0)
        && reg.sp == cur_sp)
      function_returning = true;

    cmdClearScreen();
    cmdNext();

    if (ctrlcflag)
      break;
  }
  //outputFlag = true;
  cmdDisassemble();
}

// check symbol-map for value. If not found there, just return
// the hex-value of the string
int get_sym_value(char* token)
{
  int deref_cnt = 0;
  int addr = 0;

  // '#' prefix = decimal value
  if (token[0] == '#') {
    sscanf(token+1, "%d", &addr);
    return addr;
  }
  // '%' prefix = binary value
  else if (token[0] == '%') {
    addr = parseBinaryString(token+1);
    return addr;
  }

  // check if we're de-referencing a 16-bit pointer
  while (token[0] == '*')
  {
    deref_cnt++;
    token++;
  }

  // if token starts with ":", then let's assume it is
  // for a line number of the current file
  if (token[0] == ':')
  {
    int lineno = 0;
    sscanf(&token[1], "%d", &lineno);
    type_fileloc* fl = find_lineno_in_list(lineno);
    if (!cur_file_loc)
    {
      printf("- Current source file unknown\n");
      return -1;
    }
    if (!fl)
    {
      printf("- Could not locate code at \"%s:%d\"\n", cur_file_loc->file, lineno);
      return -1;
    }
    addr = fl->addr;
    return addr;
  }

  // otherwise assume it is a symbol (which will fall back to a raw address anyway)
  type_symmap_entry* sme = find_in_symmap(token);
  if (sme != NULL)
  {
    addr = sme->addr;
  }
  else
  {
    sscanf(token, "%X", &addr);
  }

  while (deref_cnt != 0)
  {
    mem_data mem = get_mem(addr, false);
    addr = mem.b[0] + (mem.b[1] << 8);
    deref_cnt--;
  }

  return addr;
}

void print_byte_at_addr(char* token, int addr, bool useAddr28, bool show_decimal, bool show_char, bool show_binary)
{
  mem_data mem = get_mem(addr, useAddr28);

  if (show_decimal)
    printf(" %s: /d %d\n", token, mem.b[0]);
  else if (show_char) {
    printf(" %s: /c '", token);
    print_char(mem.b[0]);
    printf("'\n");
  }
  else if (show_binary) {
    printf(" %s: /b %%%s\n", token, toBinaryString(mem.b[0], NULL));
  }
  else
    printf(" %s: %02X\n", token, mem.b[0]);
}

void print_byte(char *token, bool useAddr28, bool show_decimal, bool show_char, bool show_binary)
{
  int addr = get_sym_value(token);

  print_byte_at_addr(token, addr, useAddr28, show_decimal, show_char, show_binary);
}

void cmdPrintByte(void)
{
  bool show_decimal = false;
  bool show_char = false;
  bool show_binary = false;
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    else if (strcmp(token, "/c") == 0) {
      show_char = true;
      token = strtok(NULL, " ");
    }
    else if (strcmp(token, "/b") == 0) {
      show_binary = true;
      token = strtok(NULL, " ");
    }
    if (token != NULL)
      print_byte(token, false, show_decimal, show_char, show_binary);
  }
}

void cmdPrintMByte(void)
{
  bool show_decimal = false;
  bool show_char = false;
  bool show_binary = false;

  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    else if (strcmp(token, "/c") == 0) {
      show_char = true;
      token = strtok(NULL, " ");
    }
    else if (strcmp(token, "/b") == 0) {
      show_binary = true;
      token = strtok(NULL, " ");
    }
    if (token != NULL)
      print_byte(token, true, show_decimal, show_char, show_binary);
  }
}

void print_word_at_address(char* token, int addr, bool useAddr28, bool show_decimal)
{
  mem_data mem = get_mem(addr, useAddr28);

  if (show_decimal)
    printf(" %s: /d %d\n", token, (mem.b[1]<<8) + mem.b[0]);
  else
    printf(" %s: %02X%02X\n", token, mem.b[1], mem.b[0]);
}

void print_word(char* token, bool useAddr28, bool show_decimal)
{
  int addr = get_sym_value(token);

  print_word_at_address(token, addr, useAddr28, show_decimal);
}

void cmdPrintWord(void)
{
  bool show_decimal = false;
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    if (token != NULL)
      print_word(token, false, show_decimal);
  }
}

void cmdPrintMWord(void)
{
  bool show_decimal = false;
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    if (token != NULL)
      print_word(token, true, show_decimal);
  }
}

void print_dword_at_address(char* token, int addr, bool useAddr28, bool show_decimal)
{
  mem_data mem = get_mem(addr, useAddr28);

  if (show_decimal)
    printf(" %s: /d %d\n", token, (mem.b[3]<<24) + (mem.b[2]<<16) + (mem.b[1]<<8) + mem.b[0]);
  else
    printf(" %s: %02X%02X%02X%02X\n", token, mem.b[3], mem.b[2], mem.b[1], mem.b[0]);
}

void print_dword(char* token, bool useAddr28, bool show_decimal)
{
  int addr = get_sym_value(token);

  print_dword_at_address(token, addr, useAddr28, show_decimal);
}

void cmdPrintDWord(void)
{
  bool show_decimal = false;
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    if (token != NULL)
      print_dword(token, false, show_decimal);
  }
}

void cmdPrintMDWord(void)
{
  bool show_decimal = false;
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    if (token != NULL)
      print_dword(token, true, show_decimal);
  }
}

void print_qword_at_address(char* token, int addr, bool useAddr28, bool show_decimal)
{
  mem_data mem = get_mem(addr, useAddr28);

  if (show_decimal)
    printf(" %s: /d decimal not supported for QWORD\n", token);
  else
    printf(" %s: %02X%02X%02X%02X%02X%02X%02X%02X\n", token,
        mem.b[7], mem.b[6], mem.b[5], mem.b[4],
        mem.b[3], mem.b[2], mem.b[1], mem.b[0]);
}

void print_qword(char* token, bool useAddr28, bool show_decimal)
{
  int addr = get_sym_value(token);

  print_qword_at_address(token, addr, useAddr28, show_decimal);
}

double get_float_from_int_array(unsigned int* arr)
{
  int exp = arr[0] - 128;
  int sign = arr[1] >= 128 ? -1 : 1;
  double mantissa = ((arr[1] | 0x80) << 24) + (arr[2] << 16) + (arr[3] >> 8) + arr[4];
  mantissa /= pow(2, 32);
  mantissa *= sign;
  double val = mantissa * pow(2, exp);
  if (arr[0] == 0) {
    val = 0;
  }

  return val;
}

double get_float_at_addr(int addr, bool useAddr28)
{
  mem_data mem = get_mem(addr, useAddr28);

  return get_float_from_int_array(mem.b);
}

void print_float(char* token, bool useAddr28)
{
  int addr = get_sym_value(token);

  double val = get_float_at_addr(addr, useAddr28);

  printf(" %s: %g\n", token, val);
}


void cmdPrintQWord(void)
{
  bool show_decimal = false;
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    if (token != NULL)
      print_qword(token, false, show_decimal);
  }
}

void cmdPrintMQWord(void)
{
  bool show_decimal = false;
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    if (token != NULL)
      print_qword(token, true, show_decimal);
  }
}

void print_str_maxlen(char* token, int maxlen, bool useAddr28)
{
  int addr = get_sym_value(token);
  static char string[2048] = { 0 };

  int cnt = 0;
  string[0] = '\0';

  while (1)
  {
    mem_data mem = get_mem(addr+cnt, useAddr28);

    for (int k = 0; k < 16; k++)
    {
      // If string is over 100 chars, let's truncate it, for safety...
      if (cnt > 100)
      {
        string[cnt++] = '.';
        string[cnt++] = '.';
        string[cnt++] = '.';
        mem.b[k] = 0;
      }
      if (cnt == maxlen)
      {
        mem.b[k] = 0;
      }

      string[cnt] = mem.b[k];

      if (string[cnt] >= '\xC0' && string[cnt] <= '\xDF')
        string[cnt] -= 96;

      if (mem.b[k] == 0)
      {
        printf(" %s: \"%s\"\n", token, string);
        return;
      }
      cnt++;
    }
  }
}

void print_string(char* token, bool useAddr28)
{
  print_str_maxlen(token, -1, useAddr28);
}


unsigned int mem_0f700[2100];

void add_symbol_if_not_exist(char* name, int addr)
{
  char sval[10];
  sprintf(sval,"%04X", addr);

  type_symmap_entry* sme = find_in_symmap(name);
  if (sme == NULL)
  {
      // grab a new symbol name?
      type_symmap_entry sme_new;
      sme_new.addr = addr;
      sme_new.sval = sval;
      sme_new.symbol = name;
      add_to_symmap(sme_new);
  }
}

void scan_single_letter_vars(char *token)
{
  char name[5];
  if (token == NULL)
  {
    printf("single letter vars:\n");
    printf("------------------:\n");
  }

  for (int k = 'a'; k <= 'z'; k++)
  {
    int byte_val = mem_0f700[0xfd00 - 0xf700 + (k-'a')];
    int int_val  = (mem_0f700[0xfd20 - 0xf700 + (k-'a')*2] << 8) +
                   mem_0f700[0xfd20 - 0xf700 + (k-'a')*2 + 1];

    int str_len  = mem_0f700[0xfd60 - 0xf700 + (k-'a')*3];
    int str_ptr  = mem_0f700[0xfd60 - 0xf700 + (k-'a')*3 + 1] +
                   (mem_0f700[0xfd60 - 0xf700 + (k-'a')*3 + 2] << 8);

    double float_val = get_float_at_addr(0xfe00 + (k-'a')*5, true);

    if (token == NULL) {
      printf("%c& = %d\t%c%% = %d\t%c = %-10g\t%c$ = (len:%d) ", k-32, byte_val, k-32, int_val, k-32, float_val, k-32, str_len);
      char sval[10];
      sprintf(sval, "%06X", 0x10000 + str_ptr);
      print_str_maxlen(sval, str_len, 1);
    }
    else {
      // todo: logic to compare varname against given token
      if (token[0] == (k-32) && token[1] == '&')
        printf("%c& = %d\n", (k-32), byte_val);
      if (token[0] == (k-32) && token[1] == '%')
        printf("%c%% = %d\n", (k-32), int_val);
      if (token[0] == (k-32) && token[1] == '\0')
        printf("%c = %-10g\n", (k-32), float_val);
      if (token[0] == (k-32) && token[1] == '$')
      {
        printf("%c$ = (len:%d) ", (k-32), int_val);
        char sval[10];
        sprintf(sval, "%06X", 0x10000 + str_ptr);
        print_str_maxlen(sval, str_len, 1);
      }

    }

    sprintf(name, "~%c&", k-32);
    add_symbol_if_not_exist(name, 0xfd00 + (k-'a'));
    sprintf(name, "~%c%%", k-32);
    add_symbol_if_not_exist(name, 0xfd20 + (k-'a')*2);
    sprintf(name, "~%c$", k-32);
    add_symbol_if_not_exist(name, 0xfd60 + (k-'a')*3);
    sprintf(name, "~%c", k-32);
    add_symbol_if_not_exist(name, 0xfe00 + (k-'a')*5);
  }
  printf("\n");
}

void scan_two_letter_vars(char *token)
{
  char name[5];
  if (token == NULL)
  {
    printf("two letter vars:\n");
    printf("---------------:\n");
  }

  int addr = 0x0f700;
  int maxaddr = 0x0fd00;
  int cnt = 0;
  while(addr + cnt < maxaddr)
  {
    char varnam1 = (char)mem_0f700[cnt + 0];
    char varnam2 = (char)mem_0f700[cnt + 1];
    char vartype = (char)mem_0f700[cnt + 2];

    if (varnam1 == '\0' && varnam2 == '\0')
      break;

    if (vartype == '&') {
      sprintf(name, "~%c%c&", varnam1, varnam2);
      add_symbol_if_not_exist(name, 0xf700 + cnt + 3);

      int byte_val = mem_0f700[cnt + 3];
      printf("%c%c& = %d\n", varnam1, varnam2, byte_val);
    }

    if (vartype == '%') {
      sprintf(name, "~%c%c%%", varnam1, varnam2);
      add_symbol_if_not_exist(name, 0xf700 + cnt + 3);

      int int_val  = (mem_0f700[cnt + 3] << 8) +
                     mem_0f700[cnt + 4];
      if (token == NULL || (token[0] == varnam1 && token[1] == varnam2 && token[2] == '%'))
        printf("%c%c%% = %d\n", varnam1, varnam2, int_val);
    }

    if (vartype == '"') {
      sprintf(name, "~%c%c", varnam1, varnam2);
      add_symbol_if_not_exist(name, 0xf700 + cnt + 3);

      double float_val = get_float_from_int_array(&mem_0f700[cnt + 3]);
      if (token == NULL || (token[0] == varnam1 && token[1] == varnam2 && token[2] == '\0'))
        printf("%c%c = %g\n", varnam1, varnam2, float_val);
    }

    if (vartype == '$') {
      sprintf(name, "~%c%c$", varnam1, varnam2);
      add_symbol_if_not_exist(name, 0xf700 + cnt + 3);

      int str_len  = mem_0f700[cnt + 3];
      int str_ptr  = mem_0f700[cnt + 4] +
                     (mem_0f700[cnt + 5] << 8);
      if (token == NULL || (token[0] == varnam1 && token[1] == varnam2 && token[2] == '$'))
      {
        printf("%c%c$ = (len:%d) ", varnam1, varnam2, str_len);
        char sval[10];
        sprintf(sval, "%06X", 0x10000 + str_ptr);
        print_str_maxlen(sval, str_len, 1);
      }
    }

    cnt += 8;
  }

  printf("\n");
}

void read_scalar_var_mem(void)
{
  int addr = 0x0f700;
  int maxaddr = 0x0ff00;
  int cnt = 0;
  while (addr + cnt < maxaddr)
  {
    // get memory at current pc
    mem_data* multimem = get_mem28array(addr + cnt);

    for (int line = 0; line < 16; line++)
    {
      mem_data* mem = &multimem[line];

      for (int k = 0; k < 16; k++)
      {
        mem_0f700[cnt] = mem->b[k];

        cnt++;

        if (addr + cnt >= maxaddr)
          break;
      }

      if (addr + cnt >= maxaddr)
        break;
    }
  }
}

void print_basic_var(char* token)
{
  read_scalar_var_mem();
  scan_single_letter_vars(token);
  scan_two_letter_vars(token);
  // todo: scan for arrays too
}

void print_dump(type_watch_entry* watch)
{
  int count = 16; //default count

  if (watch->param1)
    sscanf(watch->param1, "%X", &count);

  printf(" %s:\n", watch->name);

  int addr = get_sym_value(watch->name);

  dump(addr, count);
}

void print_mdump(type_watch_entry* watch)
{
  int count = 16; //default count

  if (watch->param1)
    sscanf(watch->param1, "%X", &count);

  printf(" %s:\n", watch->name);

  int addr = get_sym_value(watch->name);

  mdump(addr, count);
}

void cmdPrintString(void)
{
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    print_string(token, false);
  }
}

void cmdPrintBasicVar(void)
{
  char* token = strtok(NULL, " ");
  if (token)
    strupper(token);
  print_basic_var(token);
}

void cmdPrintMString(void)
{
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    print_string(token, true);
  }
}

void cmdPrintMFloat(void)
{
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    print_float(token, true);
  }
}

void cmdClearScreen(void)
{
  printf("%s%s", KCLEAR, KPOS0_0);
}

void stop_cpu_if_running(void)
{
  int cpu_stopped = isCpuStopped();

  if (!cpu_stopped)
  {
    printf("Stopping CPU first...\n");
    serialWrite("t1\n");
    usleep(10000);
    serialRead(inbuf, BUFSIZE);
  }
}

void set_mem(int addr, mem_data mem)
{
    char str[64];
    // reset byte-values at this point
    for (int k = 0; k < 16; k++)
    {
      sprintf(str, "s777%04X %02X\n", addr+k, mem.b[k]);
      serialWrite(str);
      usleep(10000);
      serialRead(inbuf, BUFSIZE);
    }
    serialFlush();
}

void call_temp_routine(char** routine)
{
  stop_cpu_if_running();

  reg_data reg = get_regs();
  int tmppc = 0xf0;
  mem_data mem = get_mem(tmppc, false);
  serialFlush();

  usleep(10000);

  while (*routine != NULL)
  {
    oneShotAssembly(&tmppc, *routine);
    routine++;
  }

  char strjmpcmd[10];
  sprintf(strjmpcmd, "jmp $%04x", tmppc);
  oneShotAssembly(&tmppc, strjmpcmd);

  serialFlush();
  usleep(10000);

    // reset pc
    serialWrite("gf0\n");
    serialRead(inbuf, BUFSIZE);

    usleep(100000);

    // un-pause cpu to execute
    serialWrite("t0\n");

    usleep(100000);

    // pause cpu once more
    serialWrite("t1\n");

    // reset pc
    char str[64];
    sprintf(str, "g%X\n", reg.pc);
    serialWrite(str);
    serialRead(inbuf, BUFSIZE);

    set_mem(0xf0, mem);
}

void set_rom_writable(bool write_flag)
{
  if (write_flag)
    printf("Making rom writable... ($2,0000 - $3,FFFF)\n");
  else
    printf("Making rom read-only... ($2,0000 - $3,FFFF)\n");

  stop_cpu_if_running();

  char* writeable_routine[] = { "pha", "lda #$02", "sta $d641", "clv", "pla", NULL };
  char* readonly_routine[]  = { "pha", "lda #$00", "sta $d641", "clv", "pla", NULL };

  if (write_flag)
    call_temp_routine(writeable_routine);
  else
    call_temp_routine(readonly_routine);

}


void cmdRomW(void)
{
  char* token = strtok(NULL, " ");

  // if no parameter, then just toggle it
  if (token == NULL)
    romw = !romw;
  else if (strcmp(token, "1") == 0) {
    romw = true;
  }
  else if (strcmp(token, "0") == 0) {
    romw = false;
  }

  set_rom_writable(romw);
}

int find_hyppo_service_by_name(char *name)
{
  int k = 0;
  while (hyppo_services[k].name != NULL) {
    if (strcmp(name, hyppo_services[k].name) == 0) {
      return k;
    }
    k++;
  }
  return -1;
}

void cmdHyppo(void)
{
  char* token = strtok(NULL, " ");
  hyppo_det* hd;

  if (token == NULL) {
    printf("ERROR: Missing hyppo service name argument!\n");
    printf("Available services:\n");

    int k = 0;
    while (hyppo_services[k].name != NULL) {
      hd = &hyppo_services[k];
      printf("  - %s ($%04X - A = $%02X", hd->name, hd->addr, hd->a);
      if (hd->y != -1)
        printf(", Y = $%02X", hd->y);
      printf(")");
      if (hd->help != NULL)
        printf(" %s", hd->help);
      printf("\n");
      k++;
    }
    return;
  }

  int hs_idx = find_hyppo_service_by_name(token);

  if (hs_idx == -1) {
    printf("ERROR: hyppo service not found\n");
    return;
  }

  hd = &hyppo_services[hs_idx];

  if (hd->inputfn != NULL) {
    bool ret = hd->inputfn(hd);
    if (ret == false)
      return;
  }

  char ldy_cmd[16];
  char lda_cmd[16];
  char sta_cmd[16];


  if (hd->y != -1)
    sprintf(ldy_cmd, "ldy #$%02X", hd->y);
  else
    sprintf(ldy_cmd, "nop");

  sprintf(lda_cmd, "lda #$%02X", hd->a);
  sprintf(sta_cmd, "sta $%04X", hd->addr);

  char* hyppo_routine[] = { "pha", "phx", "phy", "phz", "php", ldy_cmd, lda_cmd, sta_cmd, "clv", NULL };

  for (int k = 5; hyppo_routine[k] != NULL; k++) {
    printf("  %s\n", hyppo_routine[k]);
  }

  call_temp_routine(hyppo_routine);

  reg_data reg = get_regs();

  char* cleanup_routine[] = { "plp", "plz", "ply", "plx", "pla", NULL };
  call_temp_routine(cleanup_routine);

  show_regs(&reg);

  if (hd->outputfn != NULL)
    hd->outputfn(&reg);
}


void cmdAutoClearScreen(void)
{
  char* token = strtok(NULL, " ");

  // if no parameter, then just toggle it
  if (token == NULL)
    autocls = !autocls;
  else if (strcmp(token, "1") == 0)
    autocls = true;
  else if (strcmp(token, "0") == 0)
    autocls = false;

  printf(" - autocls is turned %s.\n", autocls ? "on" : "off");
}

void cmdSetBreakpoint(void)
{
  char* token = strtok(NULL, " ");
  char str[100];

  if (token != NULL)
  {
    int addr = get_sym_value(token);

    if (addr == -1)
      return;

    printf("- Setting hardware breakpoint to $%04X\n", addr);

    sprintf(str, "b%04X\n", addr);
    serialWrite(str);
    serialRead(inbuf, BUFSIZE);
  }
}

bool inHypervisorMode(void)
{
  // get current register values
  reg_data reg = get_regs();

  if (reg.maph == 0x3F00)
    return true;

  return false;
}

void clearSoftBreak(void)
{
  char str[100];

  // stop the cpu
  serialWrite("t1\n");
  usleep(10000);
  serialRead(inbuf, BUFSIZE);

  bool in_hv = inHypervisorMode();
  if (in_hv)
    softbrkaddr |= 0xfff0000;

  // inject JMP command to loop over itself
  if (softbrkaddr > 0xffff)
    sprintf(str, "s%04X %02X %02X %02X\n", softbrkaddr, softbrkmem[0], softbrkmem[1], softbrkmem[2]);
  else
    sprintf(str, "s777%04X %02X %02X %02X\n", softbrkaddr, softbrkmem[0], softbrkmem[1], softbrkmem[2]);
  serialWrite(str);
  serialRead(inbuf, BUFSIZE);
  softbrkaddr = 0;
}

void setSoftBreakpoint(int addr)
{
  char str[100];

  softbrkaddr = addr;

  int cpu_stopped = isCpuStopped();

  if (!cpu_stopped)
  {
    serialWrite("t1\n");
    usleep(10000);
    serialRead(inbuf, BUFSIZE);
  }

  bool in_hv = inHypervisorMode();
  if (in_hv)
    addr |= 0xfff0000;

  // user manually enforcing a hypervisor breakpoint? (or any other >64kb for that matter)
  if (addr > 0xffff)
    in_hv = true;

  mem_data mem = get_mem(addr, in_hv);
  softbrkmem[0] = mem.b[0];
  softbrkmem[1] = mem.b[1];
  softbrkmem[2] = mem.b[2];

  // inject JMP command to loop over itself
  if (in_hv)
    sprintf(str, "s%04X %02X %02X %02X\n", addr, 0x4C, addr & 0xff, (addr >> 8) & 0xff);
  else
    sprintf(str, "s777%04X %02X %02X %02X\n", addr, 0x4C, addr & 0xff, (addr >> 8) & 0xff);

  serialWrite(str);
  serialRead(inbuf, BUFSIZE);

  if (!cpu_stopped)
  {
    serialWrite("t0\n");
    usleep(100000);
    serialRead(inbuf, BUFSIZE);
  }

  // sprintf(str, "b%04X\n", addr);
  // serialWrite(str);
  // serialRead(inbuf, BUFSIZE);
}

void cmdSetSoftwareBreakpoint(void)
{
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    int addr = get_sym_value(token);

    if (addr == -1)
      return;

    printf("- Setting software breakpoint to $%04X\n", addr);

    setSoftBreakpoint(addr);
  }
}

void cmd_watch(type_watch type)
{
  bool show_decimal = false;
  bool show_char = false;

  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (strcmp(token, "/d") == 0) {
      show_decimal = true;
      token = strtok(NULL, " ");
    }
    if (strcmp(token, "/c") == 0) {
      show_char = true;
      token = strtok(NULL, " ");
    }

    if (find_in_watchlist(type, token))
    {
      printf("watch already exists!\n");
      return;
    }

    type_watch_entry we;
    we.type = type;
    we.show_decimal = show_decimal;
    we.show_char = show_char;
    we.name = token;
    we.param1 = NULL;

    token = strtok(NULL, " ");
    if (token != NULL)
    {
      if (type == TYPE_DUMP || type == TYPE_MDUMP) {
        we.param1 = token;
        token = strtok(NULL, " ");
        if (token != NULL)
        {
          we.name = token;
        }
      }

      if (token != NULL)
      {
        // grab a new symbol name?
        type_symmap_entry sme;
        sme.addr = get_sym_value(we.name);
        sme.sval = we.name;
        sme.symbol = token;
        add_to_symmap(sme);
        we.name = token;
      }
    }

    add_to_watchlist(we);

    if (we.param1 != NULL)
      printf("watch added! (%s : %s %s)\n", type_names[type], we.name, we.param1);
    else
      printf("watch added! (%s : %s)\n", type_names[type], we.name);
  }
}

void cmdWatchByte(void)
{
  cmd_watch(TYPE_BYTE);
}

void cmdWatchWord(void)
{
  cmd_watch(TYPE_WORD);
}

void cmdWatchDWord(void)
{
  cmd_watch(TYPE_DWORD);
}

void cmdWatchQWord(void)
{
  cmd_watch(TYPE_QWORD);
}

void cmdWatchString(void)
{
  cmd_watch(TYPE_STRING);
}

void cmdWatchDump(void)
{
  cmd_watch(TYPE_DUMP);
}

void cmdWatchMByte(void)
{
  cmd_watch(TYPE_MBYTE);
}

void cmdWatchMWord(void)
{
  cmd_watch(TYPE_MWORD);
}

void cmdWatchMDWord(void)
{
  cmd_watch(TYPE_MDWORD);
}

void cmdWatchMQWord(void)
{
  cmd_watch(TYPE_MQWORD);
}

void cmdWatchMString(void)
{
  cmd_watch(TYPE_MSTRING);
}

void cmdWatchMFloat(void)
{
  cmd_watch(TYPE_MFLOAT);
}

void cmdWatchMDump(void)
{
  cmd_watch(TYPE_MDUMP);
}

void cmdWatches(void)
{
  type_watch_entry* iter = lstWatches;
  int cnt = 0;

  printf("---------------------------------------\n");

  while (iter != NULL)
  {
    cnt++;

    printf("#%d: %s ", cnt, type_names[iter->type]);

    switch (iter->type)
    {
      case TYPE_BYTE:   print_byte(iter->name, false, iter->show_decimal, iter->show_char, iter->show_binary);   break;
      case TYPE_WORD:   print_word(iter->name, false, iter->show_decimal);   break;
      case TYPE_DWORD:  print_dword(iter->name, false, iter->show_decimal);  break;
      case TYPE_QWORD:  print_qword(iter->name, false, iter->show_decimal);  break;
      case TYPE_STRING: print_string(iter->name, false); break;
      case TYPE_DUMP:   print_dump(iter);         break;

      case TYPE_MBYTE:   print_byte(iter->name, true, iter->show_decimal, iter->show_char, iter->show_binary);   break;
      case TYPE_MWORD:   print_word(iter->name, true, iter->show_decimal);   break;
      case TYPE_MDWORD:  print_dword(iter->name, true, iter->show_decimal);  break;
      case TYPE_MQWORD:  print_qword(iter->name, true, iter->show_decimal);  break;
      case TYPE_MSTRING: print_string(iter->name, true); break;
      case TYPE_MDUMP:   print_mdump(iter);        break;
      case TYPE_MFLOAT:  print_float(iter->name, true); break;
    }

    iter = iter->next;
  }

  if (cnt == 0)
    printf("no watches in list\n");
  printf("---------------------------------------\n");
}

void cmdDeleteWatch(void)
{
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    // user wants to delete all watches?
    if (strcmp(token, "all") == 0)
    {
      // TODO: add a confirm yes/no prompt...
      outputFlag = false;
      while (delete_from_watchlist(1))
        ;
      outputFlag = true;
      printf("deleted all watches!\n");
    }
    else
    {
      int wnum;
      int n = sscanf(token, "%d", &wnum);

      if (n == 1)
      {
        delete_from_watchlist(wnum);
      }
    }
  }
}


void cmdAutoWatch(void)
{
  char* token = strtok(NULL, " ");

  // if no parameter, then just toggle it
  if (token == NULL)
    autowatch = !autowatch;
  else if (strcmp(token, "1") == 0)
    autowatch = true;
  else if (strcmp(token, "0") == 0)
    autowatch = false;

  printf(" - autowatch is turned %s.\n", autowatch ? "on" : "off");
}

void cmdAutoLocals(void)
{
  char* token = strtok(NULL, " ");

  // if no parameter, then just toggle it
  if (token == NULL)
    autolocals = !autolocals;
  else if (strcmp(token, "1") == 0)
    autolocals = true;
  else if (strcmp(token, "0") == 0)
    autolocals = false;

  printf(" - autolocals is turned %s.\n", autolocals ? "on" : "off");
}

char* get_rom_chunk_name(int mb, int addr)
{
  static char* empty = "";

  addr += (mb << 20);

  for (int k = 0; rom_chunks[k].addr != 0; k++) {
    if (rom_chunks[k].addr == addr) {
      return rom_chunks[k].name;
    }
  }
  return empty;
}

void cmdMapping(void)
{
  int reg_01 = peek(0x01);  // only visible in 16-bit cpu context (not in 28-bit address space, so don't use mpeek)
  int reg_d030 = mpeek(0xffd3030);

  stop_cpu_if_running();

  mem_data orig_mem = get_mem(0x0200, false); // preserve memoryw here mapping info written

  char* getmapping_routine[] = { "pha", "phy", "ldy #$02", "lda #$74", "sta $d640", "clv", "ply", "pla", NULL };

  call_temp_routine(getmapping_routine);

  int mb_mapl = mpeek(0x0204);
  int mb_maph = mpeek(0x0205);

  set_mem(0x0200, orig_mem);

  reg_data reg = get_regs();
  printf("MAPH = %04X (MB_H = %02X)  :  MAPL = %04X (MB_L = %02X)\n", reg.maph, mb_maph, reg.mapl, mb_mapl);

  printf("\n");

  printf("\"MAP\" mechanism (overrides $D030 and $01)\n");
  printf("===============\n");
  int mapl_blocks = reg.mapl >> 12;
  int mapl_offset = (reg.mapl & 0x0fff) << 8;

  int maph_blocks = reg.maph >> 12;
  int maph_offset = (reg.maph & 0x0fff) << 8;

  printf("MAPL\n");
  printf("----\n");
  for (int k = 0; k < 7; k++) {
    if (mapl_blocks & (1<<k)) {
      int offset = mapl_offset + 0x2000 * k;
      printf("- $%04X <-- $%1X,%04X %s\n", 0x2000 * k,
         offset >> 16, offset & 0xffff, get_rom_chunk_name(mb_mapl, offset));
    }
  }
  printf("\n");

  printf("MAPH\n");
  printf("----\n");
  for (int k = 0; k < 7; k++) {
    if (maph_blocks & (1<<k)) {
      int offset = maph_offset + 0x8000 + 0x2000 * k;
      printf("- $%04X <-- $%1X,%04X %s\n", 0x8000 + 0x2000 * k,
          offset >> 16, offset & 0xffff, get_rom_chunk_name(mb_maph, offset));
    }
  }
  printf("\n");

  printf("$D030 register\n");
  printf("==============\n");
  if (reg_d030 & 0x08)
    printf("- $8000 <-- $2,8000 %s\n", get_rom_chunk_name(0, 0x28000));
  if (reg_d030 & 0x10)
    printf("- $A000 <-- $2,A000 %s\n", get_rom_chunk_name(0, 0x2a000));
  if (reg_d030 & 0x20)
    printf("- $C000 <-- $2,C000 %s\n", get_rom_chunk_name(0, 0x2c000));
  if (reg_d030 & 0x80)
    printf("- $E000 <-- $2,E000 %s\n", get_rom_chunk_name(0, 0x2e000));
  printf("\n");

  printf("$01 register\n");
  printf("===========\n");

  if ((reg_01 & 0x03) == 0x03)
    printf("- $A000 <-- $2,A000 (C64 BASIC)\n");
  if ((reg_01 & 0x04) && (reg_01 & 0x03))
    printf("- $D000 <-- I/O\n");
  else if (reg_01 & 0x03)
    printf("- $D000 <-- $2,D000 (C64 CHARSET)\n");
  if (reg_01 & 0x02)
    printf("- $E000 <-- $2,E000 (C64 KERNAL)\n");
  printf("\n");
}

void cmdPetscii(void)
{
  char* token = strtok(NULL, " ");

  // if no parameter, then just toggle it
  if (token == NULL)
    petscii = !petscii;
  else if (strcmp(token, "1") == 0)
    petscii = true;
  else if (strcmp(token, "0") == 0)
    petscii = false;

  printf(" - petscii is turned %s.\n", petscii ? "on" : "off");
}

void cmdFastMode(void)
{
#ifdef __CYGWIN__
  printf("Command not available under Cygwin (only capable of 2,000,000bps)\n");
#else
  char* token = strtok(NULL, " ");

  // if no parameter, then just toggle it
  if (token == NULL)
    fastmode = !fastmode;
  else if (strcmp(token, "1") == 0)
    fastmode = true;
  else if (strcmp(token, "0") == 0)
    fastmode = false;

  serialBaud(fastmode);

  printf(" - fastmode is turned %s.\n", fastmode ? "on" : "off");
#endif
}

void cmdScope(void)
{
  char* token = strtok(NULL, " ");

  if (token == NULL)
    return;
  sscanf(token, "%d", &dis_scope);

  if (autocls)
    cmdClearScreen();
  cmdDisassemble();
}

void cmdOffs(void)
{
  char* token = strtok(NULL, " ");

  if (token == NULL) {
    dis_offs = 0;
  }
  else {
    sscanf(token, "%d", &dis_offs);
  }

  if (autocls)
    cmdClearScreen();
  cmdDisassemble();
}

int parseBinaryString(char* str)
{
  int val = 0;
  int weight = 1;
  for (int k = strlen(str)-1; k >=0; k--)
  {
    if (str[k] == '1')
      val += weight;
    weight <<= 1;
  }
  return val;
}

char* toBinaryString(int val, poke_bitfield_info* bfi)
{
  static char str[128] = "";
  str[0] = '\0';

  int maxbit = (1 << 31);
  if (val/65536 == 0)
    maxbit = (1 << 15);
  if (val/256 == 0)
    maxbit = (1 << 7);

  int bitcnt = 0;
  for (int k = maxbit; k > 0; k >>= 1)
  {
    if (bfi != NULL) {
      if (k == 1 << (bfi->start_bit + bfi->num_bits - 1)) {
        strcat(str, KINV);
      }
    }

    if (val & k)
      strcat(str, "1");
    else
      strcat(str, "0");

    if (bfi != NULL) {
      if (k == 1 << (bfi->start_bit)) {
        strcat(str, KINV_OFF);
      }
    }

    bitcnt++;
    if ( (bitcnt % 4) == 0)
    {
      strcat(str, " ");
    }
  }
  return str;
}

void cmdPrintValue(void)
{
  char* strVal = strtok(NULL, " ");

  if (strVal == NULL)
  {
    printf("Missing <value> parameter!\n");
    return;
  }

  int val = 0;
  val = get_sym_value(strVal);

  char charval[64] = "";
  int origval = val;
  if (val >= 32 && val < 256) {
    if (val >= 192 && val <= 223)
      val -= 96;

    sprintf(charval, "(char='%c')", val);
    val = origval;
  }

  printf("  $%04X  /  #%d  /  %%%s %s\n", val, val, toBinaryString(val, NULL), charval);
}

int isCpuStopped(void)
{
  int cnt = 0;
  int cur_pc = -1;
  int same_cnt = 0;
  while ( cnt < 10 )
  {
    // get current register values
    reg_data reg = get_regs();

    usleep(10000);

    if (reg.pc == cur_pc)
    {
      same_cnt++;
      if (same_cnt == 5)
        return 1;
    }
    else
    {
      same_cnt = 0;
      cur_pc = reg.pc;
    }
    cnt++;
  }

  return 0;
}

int doOneShotAssembly(char* strCommand)
{
  int numbytes;
  char str[128];

  int cpu_stopped = isCpuStopped();

  if (!cpu_stopped)
  {
    printf("Stopping CPU first...\n");
    serialWrite("t1\n");
    usleep(10000);
    serialRead(inbuf, BUFSIZE);
  }

  reg_data reg = get_regs();
  int tmppc = 0xf0;

  usleep(10000);

  // get memory at 0xf0 (usually always 'writable' memory in zero-page)
  // serialFlush();
  mem_data mem = get_mem(tmppc, false);
  serialFlush();
  // serialRead(inbuf, BUFSIZE);

  usleep(10000);

  numbytes = oneShotAssembly(&tmppc, strCommand);
  serialFlush();

  usleep(10000);

  if (numbytes > 0)
  {
    // reset pc
    serialWrite("gf0\n");
    serialRead(inbuf, BUFSIZE);

    usleep(100000);

    // just send an enter command to do single step
    serialWrite("\n");

    usleep(100000);

    // reset pc
    sprintf(str, "g%X\n", reg.pc);
    serialWrite(str);
    serialRead(inbuf, BUFSIZE);

    // usleep(100000);

    // reset byte-values at this point
    for (int k = 0; k < numbytes; k++)
    {
      sprintf(str, "s777%04X %02X\n", 0xf0+k, mem.b[k]);
      serialWrite(str);
      usleep(10000);
      serialRead(inbuf, BUFSIZE);
    }
  }

   serialFlush();

  if (autocls)
    cmdClearScreen();

  if (autowatch) {
    cmdWatches();
    reg_data reg = get_regs();
    show_regs(&reg);
  }

   return numbytes;
}

void find_addr_in_symmap(int addr, int eaddr)
{
  type_symmap_entry* iter = lstSymMap;

  while (iter != NULL)
  {
    if ((eaddr == -1 && addr == iter->addr) ||
        (addr <= iter->addr && iter->addr <= eaddr)) {
      printf("%s : %s\n", iter->sval, iter->symbol);
    }

    iter = iter->next;
  }
}

void cmdSymbolValue(void)
{
  char* token = strtok(NULL, " ");

  if (token != NULL)
  {
    if (token[0] == '$') {
      int addr;
      int addr2 = -1;
      sscanf(token+1, "%X", &addr);

      char* token = strtok(NULL, " ");
      if (token != NULL && token[0] == '$') {
        sscanf(token+1, "%X", &addr2);
      }

      find_addr_in_symmap(addr, addr2);
    }
    else {
      type_symmap_entry* sme = find_in_symmap(token);

      if (sme != NULL)
        printf("%s : %s\n", sme->sval, sme->symbol);
    }
  }
}

void cmdSave(void)
{
  char* strBinFile = strtok(NULL, " ");

  if (!strBinFile)
  {
    printf("Missing <binfile> parameter!\n");
    return;
  }

  char* strAddr = strtok(NULL, " ");
  if (!strAddr)
  {
    printf("Missing <addr> parameter!\n");
    return;
  }

  char* strCount = strtok(NULL, " ");
  if (!strCount)
  {
    printf("Missing <count> parameter!\n");
    return;
  }

  int addr = get_sym_value(strAddr);
  int count;
  sscanf(strCount, "%X", &count);

  int cnt = 0;
  FILE* fsave = fopen(strBinFile, "wb");
  while (cnt < count)
  {
    // get memory at current pc
    mem_data* multimem = get_mem28array(addr + cnt);

    for (int line = 0; line < 16; line++)
    {
      mem_data* mem = &multimem[line];

      for (int k = 0; k < 16; k++)
      {
        fputc(mem->b[k], fsave);

        cnt++;

        if (cnt >= count)
          break;
      }

      printf("0x%X bytes saved...\r", cnt);
      if (cnt >= count)
        break;
    }

    if (ctrlcflag)
      break;
  }

  printf("\n0x%X bytes saved to \"%s\"\n", cnt, strBinFile);
  fclose(fsave);
}

void cmdLoad(void)
{
  char* strBinFile = strtok(NULL, " ");

  if (!strBinFile)
  {
    printf("Missing <binfile> parameter!\n");
    return;
  }

  char* strAddr = strtok(NULL, " ");
  if (!strAddr)
  {
    printf("Missing <addr> parameter!\n");
    return;
  }

  int addr = get_sym_value(strAddr);

    FILE* fload = fopen(strBinFile, "rb");
  if(fload)
  {
    fseek(fload, 0, SEEK_END);
    int fsize = ftell(fload);
    rewind(fload);
    char* buffer = (char *)malloc(fsize*sizeof(char));
    if(buffer)
    {
      fread(buffer, fsize, 1, fload);

      int i = 0;
      while(i < fsize)
      {
        int outSize = fsize - i;
        if(outSize > 16) {
          outSize = 16;
        }

        put_mem28array(addr + i, (unsigned char*) (buffer + i), outSize);
        i += outSize;
      }

      free(buffer);
    }
      fclose(fload);
  }
  else
  {
    printf("Error opening the file '%s'!\n", strBinFile);
  }
}

void cmdBackTrace(void)
{
  char str[128] = { 0 };

  // get current register values
  reg_data reg = get_regs();

  disassemble_addr_into_string(str, reg.pc, false);
  if (traceframe == 0)
    printf(KINV "#0: %s\n" KNRM, str);
  else
    printf("#0: %s\n", str);

  // get memory at current pc
  int* addresses = get_backtrace_addresses();

  for (int k = 0; k < 8; k++)
  {
    disassemble_addr_into_string(str, addresses[k], false);
    if (traceframe-1 == k)
      printf(KINV "#%d: %s\n" KNRM, k+1, str);
    else
      printf("#%d: %s\n", k+1, str);
  }
}

void cmdUpFrame(void)
{
  if (traceframe == 0)
  {
    printf("Already at highest frame! (frame#0)\n");
    return;
  }

  traceframe--;

  if (autocls)
    cmdClearScreen();
  cmdDisassemble();
}

void cmdDownFrame(void)
{
  if (traceframe == 8)
  {
    printf("Already at lowest frame! (frame#8)\n");
    return;
  }

  traceframe++;

  if (autocls)
    cmdClearScreen();
  cmdDisassemble();
}

void search_range(int addr, int total, unsigned char *bytes, int length)
{
  int cnt = 0;
  bool found_start = false;
  int found_count = 0;
  int start_loc = 0;
  int results_cnt = 0;

  printf("Searching for: ");
  for (int k = 0; k < length; k++)
  {
    printf("%02X ", bytes[k]);
  }
  printf("\n");

  while (cnt < total)
  {
    // get memory at current pc
    mem_data *multimem = get_mem28array(addr + cnt);

    for (int m = 0; m < 16; m++)
    {
      mem_data mem = multimem[m];

      for (int k = 0; k < 16; k++)
      {
        if (!found_start)
        {
          if (mem.b[k] == bytes[0])
          {
            found_start = true;
            start_loc = mem.addr + k;
            found_count++;
            if (length == 1) {
              printf("%07X\n", start_loc);
              found_start = false;
              found_count = 0;
              start_loc = 0;
              results_cnt++;
            }
          }
        }
        else // matched till the end?
        {
          if (mem.b[k] == bytes[found_count])
          {
            found_count++;
            // we found a complete match?
            if (found_count == length)
            {
              printf("%07X\n", start_loc);
              found_start = false;
              found_count = 0;
              start_loc = 0;
              results_cnt++;
            }
          }
          else
          {
            found_start = false;
            start_loc = 0;
            found_count = 0;
          }
        }
      } // end for k

      cnt+=16;

      if (cnt > total)
        break;
    } // end for m

    if (ctrlcflag)
      break;
  }

  if (results_cnt == 0)
  {
    printf("None found...\n");
  }
}

void cmdSearch(void)
{
  char* strAddr = strtok(NULL, " ");
  char bytevals[64] = { 0 };
  int len = 0;

  if (strAddr == NULL)
  {
    printf("Missing <addr28> parameter!\n");
    return;
  }

  int addr = get_sym_value(strAddr);

  int total = 16;
  char* strTotal = strtok(NULL, " ");

  if (strTotal != NULL)
  {
    sscanf(strTotal, "%X", &total);
  }

  char* strValues = strtok(NULL, "\0");

  // provided a string?
  if (strValues[0] == '\"')
  {
    search_range(addr, total, (unsigned char*)&strValues[1], strlen(strValues)-2);
  }
  else
  {

    char *sval = strtok(strValues, " ");
    do
    {
      int ival;
      sscanf(sval, "%X", &ival);
      bytevals[len] = ival;
      len++;
    }
    while ( (sval = strtok(NULL, " ")) != NULL);

    search_range(addr, total, (unsigned char*)bytevals, len);
  }
}

extern int fd;

void cmdScreenshot(void)
{
  int orig_fcntl = fcntl(fd, F_GETFL, NULL);
  fcntl(fd,F_SETFL,orig_fcntl|O_NONBLOCK);
  get_video_state();
  do_screen_shot_ascii();
  fcntl(fd,F_SETFL,orig_fcntl);
}

extern int type_text_cr;
void cmdType(void)
{
  char* tok = strtok(NULL, "\0");
  int orig_fcntl = fcntl(fd, F_GETFL, NULL);
  fcntl(fd,F_SETFL,orig_fcntl|O_NONBLOCK);

  if (tok != NULL)
  {
    type_text_cr=1;
    do_type_text(tok);
  }
  else
  {
    do_type_text("-");
  }
  fcntl(fd,F_SETFL,orig_fcntl);
}

extern char pathBitstream[];
int do_ftp(char* bitstream);
extern char devSerial[];

void cmdFtp(void)
{
  int orig_fcntl = fcntl(fd, F_GETFL, NULL);
  fcntl(fd,F_SETFL,orig_fcntl|O_NONBLOCK);
  do_ftp(pathBitstream);
  fcntl(fd,F_SETFL,orig_fcntl);
  serialClose();
  serialOpen(devSerial);
}

int cmdGetCmdCount(void)
{
  return sizeof(command_details) / sizeof(type_command_details) - 1;
}

char* cmdGetCmdName(int idx)
{
  return command_details[idx].name;
}
