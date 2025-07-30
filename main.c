/* vim: set expandtab shiftwidth=2 tabstop=2: */

/**
 * m65dbg - An enhanced remote serial debugger/monitor for the mega65 project
 **/

#define _BSD_SOURCE _BSD_SOURCE
#include <stdio.h>
#include <string.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include "serial.h"
#include "commands.h"

#define VERSION "v1.00"

char *strInput = NULL;
char pathBitstream[256] = "";
char devSerial[100] = "/dev/ttyUSB1";

/**
 * retrieves a command via user input and places it in global strInput
 */
void get_command(void)
{
  strInput = readline("<dbg>");
}


extern bool fastmode;

void parse_command(void)
{
  char* token;
  bool handled = false;

  // if command is empty, then repeat last command
  if (strlen(strInput) == 0)
  {
    free(strInput);
    strInput = (char*)malloc(strlen(outbuf)+1);
    strcpy(strInput, outbuf);
  }

  // ignore no command
  if (strlen(strInput) == 0)
    return;

  // preserve a copy of original command
  strcpy(outbuf, strInput);

  // assume it might be a one-shot assembly command
  if (isValidMnemonic(strInput))
  {
    // restore original command
    strcpy(strInput, outbuf);

    if (doOneShotAssembly(strInput) > 0)
      handled = true;
  }

  // restore original command
  strcpy(strInput, outbuf);

  // tokenise command
  token = strtok(strInput, " [");

  // test for special commands provided by the m65dbg app
  if (!handled)
  {
    for (int k = 0; command_details[k].name != NULL; k++)
    {
      if (strcmp(token, command_details[k].name) == 0)
      {
        command_details[k].func();
        handled = true;
        break;
      }
    }
  }

  // if command is not handled by m65dbg, then just pass across raw command
  if (!handled)
  {
    serialWrite(outbuf);
    if (strncmp(outbuf, "!", 1) == 0)
    {
#ifndef __CYGWIN__
      fastmode = false;
      serialBaud(fastmode);
#endif
    }
    serialRead(inbuf, BUFSIZE);
    printf("%s", inbuf);
  }

  if (strInput != NULL)
  {
    free(strInput);
    strInput = NULL;
  }

}

// use ctrl-c to break out of any commands that loop (eg, finish/next)
void ctrlc_handler(int s)
{
  ctrlcflag = true;
}

extern BitfieldInfo bitfields[];
extern hyppo_det hyppo_services[];

char* seam_field_gen(const char* text, int state)
{
    static int list_index, len;
    const char *name;

    if (!state) {
        list_index = 0;
        len = strlen(text);
    }

    while ((name = bitfields[list_index++].name)) {
        if (strncmp(name, text, len) == 0) {
            return strdup(name); // Match found
        }
    }

    return NULL; // No more matches
}

char* hyppo_service_gen(const char* text, int state)
{
    static int list_index, len;
    const char *name;

    if (!state) {
        list_index = 0;
        len = strlen(text);
    }

    while ((name = hyppo_services[list_index++].name)) {
        if (strncmp(name, text, len) == 0) {
            return strdup(name); // Match found
        }
    }

    return NULL; // No more matches
}

extern int cntMapB65to11;
extern TYP_MAP_ENTRY_11_TO_B65 mapb65to11[];

char* my_generator(const char* text, int state)
{
  static int len;
  static type_symmap_entry* iter = NULL;
  static int cmd_idx = 0;
  static int k = 0;

  if( !state )
  {
    len = strlen(text);
    iter = lstSymMap;
    cmd_idx = 0;
    k = 0;
  }

  // check if it is a symbol name
  while(iter != NULL)
  {
    if (strncmp(iter->symbol, text, len) == 0 )
    {
      char *s = strdup(iter->symbol);
      iter = iter->next;
      return s;
    }

    iter = iter->next;
  }

  while (cmd_idx < cmdGetCmdCount())
  {
    if (strncmp(cmdGetCmdName(cmd_idx), text, len) == 0 )
    {
      char *s = strdup(cmdGetCmdName(cmd_idx));
      cmd_idx++;
      return s;
    }
    cmd_idx++;
  }

  for (; k < cntMapB65to11; k++)
  {
    if (strncmp(mapb65to11[k].eleven, text, len) == 0)
    {
      char *s = strdup(mapb65to11[k].eleven);
      k++;
      return s;
    }
  }

  return((char *)NULL);
}


static char** my_completion(const char * text, int start, int end)
{
    char **matches;
    int idx1, idx2, matched_chars;
    matches = (char **)NULL;
    //if( start == 0 )
    //{
    if (sscanf(rl_line_buffer, "seam[%d][%d].%n", &idx1, &idx2, &matched_chars) == 2
        && matched_chars > 0 && rl_line_buffer[matched_chars - 1] == '.') {
      matches = rl_completion_matches(text, &seam_field_gen);
    }
    else if (strncmp(rl_line_buffer, "hyppo ", 6) == 0) {
      matches = rl_completion_matches(text, &hyppo_service_gen);
    }
    else
      matches = rl_completion_matches((char*)text, &my_generator);
    //}
    //else
    //  rl_bind_key('\t',rl_insert);
    return matches;
}

void load_init_file(char* filepath)
{
  if( access( filepath, F_OK ) != -1 )
  {
    printf("Loading \"%s\"...\n", filepath);

    FILE* f = fopen(filepath, "r");
    char* line = NULL;
    size_t len = 0;

    while (getline(&line, &len, f) != -1)
    {
      // remove any newline character at end of line
      line[strcspn(line, "\n")] = 0;

      // ignore empty lines
      if (strlen(line) == 0)
        continue;

      // ignore any lines that start with '#', treat these as comments
      if (strlen(line) > 0 && line[0] == '#')
        continue;

      // execute each line
      strInput = strdup(line);
      parse_command();
    }

    if (line != NULL)
      free(line);
  }
}

/** Look for a global "~/.m65dbg_init" file.
 *  Also look for a local/project specific ".m65dbg_init" in current path
 *
 * If either exists, load it and run the commands within it.
*/
void run_m65dbg_init_file_commands()
{
  // file exists
  char* HOME = getenv("HOME");
  char filepath[256];
  sprintf(filepath, "%s/.m65dbg_init", HOME);

  load_init_file(filepath);
  load_init_file(".m65dbg_init");
}

const char *dbg_word_break_chars = " \t\n\"\\'`@$><=;|&{(*.";  // adding the '*' and '.' onto basic word break chars
const char *history_file = ".history.txt";

/**
 * main entry point of program
 *
 * argc = number of arguments
 * argv = string array of arguments
 */
int main(int argc, char** argv)
{
  rl_completer_word_break_characters = dbg_word_break_chars;
  read_history(history_file);

  signal(SIGINT, ctrlc_handler);
  rl_initialize();

  printf("m65dbg - " VERSION "\n");
  printf("======\n");

  // check parameters
  for (int k = 1; k < argc; k++)
  {
    if (strcmp(argv[k], "--help") == 0 ||
        strcmp(argv[k], "-h") == 0)
    {
      printf("--help/-h = display this help\n"
             "--device/-l </dev/tty*> = select a tty device-name to use as the serial port to communicate with the Nexys hardware\n"
             "-b <bistream.bit> = Name of bitstream file to load (needed for ftp support)\n");
      exit(0);
    }
    if (strcmp(argv[k], "--device") == 0 ||
        strcmp(argv[k], "-l") == 0)
    {
      if (k+1 >= argc)
      {
        printf("Device name for serial port is missing (e.g., /dev/ttyUSB1)\n");
        exit(0);
      }
      k++;
      strcpy(devSerial, argv[k]);
    }

    if (strcmp(argv[k], "-b") == 0)
    {
      if (k+1 >= argc)
      {
        printf("Please provide path to bitstream file\n");
        exit(0);
      }
      k++;
      strcpy(pathBitstream, argv[k]);
    }
  }

  // open the serial port
  if (!serialOpen(devSerial))
    return 1;

  printf("- Type 'help' for new commands, '?'/'h' for raw commands.\n");

  listSearch();

  run_m65dbg_init_file_commands();

  while(1)
  {
    ctrlcflag = false;

    rl_attempted_completion_function = my_completion;

    get_command();

    if (!strInput ||
        strcmp(strInput, "exit") == 0 ||
        strcmp(strInput, "quit") == 0 ||
        strcmp(strInput, "x") == 0 ||
        strcmp(strInput, "q") == 0)
    {
      write_history(history_file);
      return 0;
    }

    if (strInput && *strInput)
      add_history(strInput);

    parse_command();
  }
}
