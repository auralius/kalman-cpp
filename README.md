# kalman-cpp

## Kalman filter and extended Kalman filter implementation in C++ 

The Kalman filter implementation is located in: kf.h and kf.cpp while the extended Kalman filter implementation is located in ekf.h and ekf.cpp.

Please use cmake to build all the codes.

The steps to compile are:

```
mkdir build
cd build
cmake ..
make
```  

## Windows System  

In a Windows system, a Visual Studio solution file (VS 2008) is provided. 

## Dependencies

This library utilizes [Armadillo](http://arma.sourceforge.net). 
In linux system, you can get it from your distribution repository. 
In Windows system, all necessary precompiled libraries are provided in "windows-libs" folder. 
The contents of windows-libs.zip need to be first extracted. 
In "bin" folder there are necessary DLLs (blas and lapack) to run your programs.
Armadillo itself is very easy to use. 
More information on the Armadillo can be found [here](http://arma.sourceforge.net/docs.html).

## MATLAB m-files for plotting

MATLAB m-files for each example are provided in 'm-files' folder. Octave can also be used instead of MATLAB.

## Documentation

See documentation [here](docs/html/index.html).
