# prepare_benchmarks

Clone this package in your workspace's `src/` directory. To use the library 
in another package simply specify it as a catkin component:
``` cmake
find_package(
  catkin REQUIRED COMPONENTS
  roscppe
  prepare_benchmarks
  #...
)
```

and `#include` the header `<prepare_benchmarks.h>` in your project.
