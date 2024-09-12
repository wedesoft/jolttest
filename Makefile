CCFLAGS = -g -fPIC -Wall -Werror -DNDEBUG -DJPH_PROFILE_ENABLED -DJPH_DEBUG_RENDERER -DJPH_OBJECT_STREAM $(shell pkg-config --cflags glfw3 glew)
LDFLAGS = $(shell pkg-config --libs glfw3 glew) -lJolt

all: tumble

tumble: tumble.o
	g++ -o $@ $^ $(LDFLAGS)

clean:
	rm -f tumble *.o

.cc.o:
	g++ -c $(CCFLAGS) -o $@ $<
