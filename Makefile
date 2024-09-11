CCFLAGS = $(shell pkg-config --cflags glfw3 glew)
LDFLAGS = $(shell pkg-config --libs glfw3 glew) -lJolt

all: tumble

tumble: tumble.o
	g++ -o $@ $^ $(LDFLAGS)

clean:
	rm -f tumble *.o

.cc.o:
	g++ -c -g -Wall -Werror $(CCFLAGS) -o $@ $<
