# kalman-cpp

[![C/C++ CI](https://github.com/auralius/kalman-cpp/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/auralius/kalman-cpp/actions/workflows/c-cpp.yml)

## Kalman filter and extended Kalman filter implementation in C++ 

Implemented filters so far: 
* Kalman filter  
* Extended Kalman filter  
* Second-order extended Kalman filter  
* Unscented Kalman filter  

Please use cmake to build all the codes.

The steps to compile are:

```
mkdir build
cd build
cmake ..
make
```  

## Windows System  

In a Windows system, a Visual Studio solution file (VS 2019) is provided. 

## Dependencies

This library utilizes [Armadillo](http://arma.sourceforge.net). 
In Windows system, the armadillo library is provided in "windows-libs" folder. 
**The contents of windows-libs.zip need to be first extracted.** 
Armadillo itself is very easy to use. 
More information on the Armadillo can be found [here](http://arma.sourceforge.net/docs.html).

## blas and lapack
By default, now kalman-cpp uses blas and lapack. For Windows machine, working with blas and lapack is a messy stuff. Thus, we will use the precompiled blas and lapack from:  https://www.fi.muni.cz/~xsvobod2/misc/lapack/. 

The precompiled blas and lapack libraries are included in **windows-libs.zip**. There are four LIB files. Additionally, in "bin" folder, there are four corresponding DLL files as well. There are four files because two files are for the 32-bit platform, and the other two files are for the 64-bit platform. 

**The compiled binary must always be located in the same folder as these DLL files.**



## MATLAB m-files for plotting

MATLAB m-files for each example are provided in 'm-files' folder. Octave can also be used instead of MATLAB.

## Documentation
  
https://auralius.github.io/kalman-cpp/
