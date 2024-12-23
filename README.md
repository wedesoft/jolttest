## Chronotest

Simple examples to test the capabilities of Jolt (built under GNU/Linux).

### Dependencies

You need to build [Jolt][1] version 5.1.0 with double precision as follows:

```Shell
cd Build
./cmake_linux_clang_gcc.sh Release g++ -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DDOUBLE_PRECISION=ON \
    -DDEBUG_RENDERER_IN_DEBUG_AND_RELEASE=OFF -DPROFILER_IN_DEBUG_AND_RELEASE=OFF
cd Linux_Release
make -j `nproc`
sudo make install
cd ../..
```

Further, you need to install [GLFW][3] and [GLEW][4] for visualisation.

### Build

```Shell
make
```

### Run
### Tumbling cuboid in space

[![Tumbling cuboid in space](https://i.ytimg.com/vi/kZoc2nsGFH4/hqdefault.jpg)](https://www.youtube.com/watch?v=kZoc2nsGFH4)

```Shell
export LD_LIBRARY_PATH=/usr/local/lib
./tumble
```

### Falling stack of cuboids

[![Falling stack](https://i.ytimg.com/vi/vo4-9reTK78/hqdefault.jpg)](https://www.youtube.com/watch?v=vo4-9reTK78)

```Shell
export LD_LIBRARY_PATH=/usr/local/lib
./stack
```

### Double pendulum

[![Double pendulum](https://i.ytimg.com/vi/ITSNDQgw13U/hqdefault.jpg)](https://www.youtube.com/watch?v=ITSNDQgw13U)

```Shell
export LD_LIBRARY_PATH=/usr/local/lib
./pendulum
```

### Suspension

[![Double pendulum](https://i.ytimg.com/vi/f2Rcfzaxo9I/hqdefault.jpg)](https://www.youtube.com/watch?v=f2Rcfzaxo9I)

```Shell
export LD_LIBRARY_PATH=/usr/local/lib
./suspension
```

### Vehicle

[![Vehicle](https://i.ytimg.com/vi/LWSXWqWFKmQ/hqdefault.jpg)](https://www.youtube.com/watch?v=LWSXWqWFKmQ)

```Shell
export LD_LIBRARY_PATH=/usr/local/lib
./vehicle
```

[1]: https://github.com/jrouwe/JoltPhysics
[2]: https://github.com/jrouwe/JoltPhysics/blob/master/Build/README.md
[3]: https://www.glfw.org/
[4]: https://glew.sourceforge.net/
