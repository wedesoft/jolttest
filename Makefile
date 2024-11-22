CCFLAGS = -g -O3 -fPIC -Wall -Werror -DNDEBUG -DJPH_OBJECT_STREAM -DJPH_DOUBLE_PRECISION $(shell pkg-config --cflags glfw3 glew)
LDFLAGS = -flto=auto $(shell pkg-config --libs glfw3 glew) -lJolt

all: tumble pendulum stack suspension vehicle

tumble: tumble.o
	g++ -o $@ $^ $(LDFLAGS)

pendulum: pendulum.o
	g++ -o $@ $^ $(LDFLAGS)

stack: stack.o
	g++ -o $@ $^ $(LDFLAGS)

suspension: suspension.o
	g++ -o $@ $^ $(LDFLAGS)

vehicle: vehicle.o
	g++ -o $@ $^ $(LDFLAGS)

clean:
	rm -f tumble pendulum stack suspension vehicle *.o

.cc.o:
	g++ -c $(CCFLAGS) -o $@ $<
