# kalman-cpp

## Kalman filter and extended Kalman filter implementation in C++ 

Implemented filters so far: 
* Kalman filter  
* Extended Kalman filter  
* Second order extended Kalman filter  
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
The contents of windows-libs.zip need to be first extracted. 
In "bin" folder, there is libopenblas.dll which is necessary to run your programs. The compiled binary must be located in the same folder with libopenblas.dll.
Armadillo itself is very easy to use. 
More information on the Armadillo can be found [here](http://arma.sourceforge.net/docs.html).

## MATLAB m-files for plotting

MATLAB m-files for each example are provided in 'm-files' folder. Octave can also be used instead of MATLAB.

## Documentation

See documentation [here](https://www.notion.so/kalman-cpp-90a2225b82e14d1d83fee65edd3cc97e).

## Github page  
https://auralius.github.io/kalman-cpp/
