CCFLAGS = -g -fPIC -Wall -Werror -DNDEBUG -DJPH_PROFILE_ENABLED -DJPH_DEBUG_RENDERER -DJPH_OBJECT_STREAM -DJPH_DOUBLE_PRECISION $(shell pkg-config --cflags glfw3 glew)
LDFLAGS = -flto=auto $(shell pkg-config --libs glfw3 glew) -lJolt

all: tumble pendulum

tumble: tumble.o
	g++ -o $@ $^ $(LDFLAGS)

pendulum: pendulum.o
	g++ -o $@ $^ $(LDFLAGS)

clean:
	rm -f tumble pendulum *.o

.cc.o:
	g++ -c $(CCFLAGS) -o $@ $<
