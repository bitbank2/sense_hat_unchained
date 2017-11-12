CFLAGS=-c -Wall -O2

all: libsensehat.a

libsensehat.a: sensehat.o
	ar -rc libsensehat.a sensehat.o ;\
	sudo cp libsensehat.a /usr/local/lib ;\
	sudo cp sensehat.h /usr/local/include

sensehat.o: sensehat.h sensehat.c
	$(CC) $(CFLAGS) sensehat.c

clean:
	rm *.o libsensehat.a

