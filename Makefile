CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra
PREFIX ?= /usr/local

cHeat: cHeat.c
	$(CC) $(CFLAGS) -o cHeat cHeat.c -lm

install: cHeat
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 755 cHeat $(DESTDIR)$(PREFIX)/bin/cHeat

clean:
	rm -f cHeat
