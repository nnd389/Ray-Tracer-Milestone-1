# Ray-Tracer-Milestone-1
This is the Ray Tracer Milestone 1 project for Nina De La Torre and Tyler O'Brien

## Build commands
From the project root directory run:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
```
## Running the RayTracer
```bash
./build/bin/ray -r 5 <input.ray> <output.png>
```

