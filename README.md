# PcbRouter

Printed Circuit Board (PCB) router

### Prerequisites

- GCC >=4.8
- G++ >= 4.8
- SWIG >= 2.0
- Boost >= 1.6
- CMake >= 3.1
- Current support for .kicad_pcb format derived from KiCad v5.1.2

### Installing

Clone
```
git clone --recurse-submodules https://github.com/The-OpenROAD-Project/PcbRouter.git
```

Build
```
mkdir build
cd build
cmake ..
make
```

Run
```
mkdir output
./bin/pcbrouter [input_filename].kicad_pcb 
```

Output
```
Routing results are in the output/ folder
```

## License
  * BSD-3-clause License [[Link]](LICENSE)
