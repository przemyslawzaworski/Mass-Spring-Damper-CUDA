Mass Spring Damper (CUDA 2D multipass fluid particle simulation) - tiny framework written by P.Z.

Reference: https://www.shadertoy.com/view/Wt3GRM

Compile with Visual Studio command line:
nvcc -o demo.exe demo.cu -arch=sm_30 user32.lib gdi32.lib

Edit lines 3-4 to define custom screen resolution.
Move and press LMB to interact.

Tested with NVCC version 9.0.176 and RTX 2070.

Press ESC to exit.

![alt text](screenshot.gif)
