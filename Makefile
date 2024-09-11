CCFLAGS = -DEIGEN_MAX_ALIGN_BYTES=32 $(shell pkg-config --cflags glfw3 glew)
LDFLAGS = $(shell pkg-config --libs glfw3 glew)

all: tumble

tumble: tumble.o
	g++ -o $@ $^ $(LDFLAGS)

clean:
	rm -f tumble *.o

.cc.o:
	g++ -c -g -Wall -Werror $(CCFLAGS) -o $@ $<
