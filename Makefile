CCFLAGS = -g -O3 -fPIC -Wall -Werror -DNDEBUG -DJPH_OBJECT_STREAM -DJPH_DOUBLE_PRECISION $(shell pkg-config --cflags glfw3 glew)
LDFLAGS = -flto=auto $(shell pkg-config --libs glfw3 glew) -lJolt

all: tumble pendulum stack suspension vehicle

tumble: tumble.o
	g++ -o $@ $^ $(LDFLAGS)
	strip $@

pendulum: pendulum.o
	g++ -o $@ $^ $(LDFLAGS)
	strip $@

stack: stack.o
	g++ -o $@ $^ $(LDFLAGS)
	strip $@

suspension: suspension.o
	g++ -o $@ $^ $(LDFLAGS)
	strip $@

vehicle: vehicle.o
	g++ -o $@ $^ $(LDFLAGS)
	strip $@

clean:
	rm -f tumble pendulum stack suspension vehicle *.o

.cc.o:
	g++ -c $(CCFLAGS) -o $@ $<
