# Refreshed my memory on how to write makefiles via this site:
# - http://mrbook.org/blog/tutorials/make/

# Add some logic to detect cygwin
TEST:=$(shell test -d /cygdrive && echo cygwin)
ifneq "$(TEST)" ""
  LDFLAGS=-L/usr/bin -lreadline7
else
  LDFLAGS=-lreadline
endif

CC=gcc
CFLAGS=-c -Wall -g -std=c99
COPT=	-I/opt/homebrew/include -L/opt/homebrew/lib -I /usr/include
LDFLAGS+=-lpng -lm
SOURCES=main.c serial.c commands.c gs4510.c screen_shot.c m65.c mega65_ftp.c ftphelper.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=m65dbg

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(COPT) $(OBJECTS) $(LDFLAGS) -o $@

.c.o:
	$(CC) $(COPT) $(CFLAGS) $< -o $@

install: m65dbg
	ln -s $(CURDIR)/m65dbg /usr/local/bin/m65dbg

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

zip: FORCE
	rm -f m65dbg.zip
	rm -rf pkg/
	mkdir pkg
	cp m65dbg.exe pkg/
	cp /bin/cygpng16-16.dll pkg/
	cp /bin/cygwin1.dll pkg/
	cp /bin/cygreadline7.dll pkg/
	cp /bin/cygz.dll pkg/
	cp /bin/cygwin1.dll pkg/
	cp /bin/cygncursesw-10.dll pkg/
	7z a m65dbg.zip ./pkg/*

FORCE:
