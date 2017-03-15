# FrameFab


## 1. Overview

This code implements a sequence generation algorithm for robotic spatial printing, which is addressed in the following paper:

Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu. FrameFab: Robotic Fabrication of Frame Shapes.
ACM Trans. Graph. 35, 6, 2016.

## 2. Dependencies

This code has been tested in Windows (built with VS2013) and Ubuntu 14.04-LTS.

 - Qt & OpenGL, GLUT
 - LAPACK & BLAS 
 - MOSEK
 - Eigen3
 - [Geometric Tools Engine](http://www.geometrictools.com/Downloads/Downloads.html)

### Useful tools for installation

- git
- Cmake (default version 2.8)

## 3. Installation Instructions

### Usual clone and Cmake procedure

- Clone Framefab from [github](https://github.com/yijiangh/FrameFab)
> $ git clone https://github.com/yijiangh/FrameFab

- Install git, cmake, Eigen, BLAS, LAPACK using your package manager. In Ubuntu, that's:
> $ sudo apt-get install git cmake libeigen3-dev libblas-dev liblapack-dev

- Mosek
	- download and install Mosek from [https://mosek.com/](https://mosek.com/). You'll need to request a license. It's free for academic use.
	- set the `PATH` environment variable to full path by adding `export PATH=<MSKHOME>/mosek/8/tools/platform/linux64x86/bin:$PATH`, e.g. in `~/.bashrc` in Ubuntu machine (`MSKHOME` denote the directory in which MOSEK is unpacked and installed. Version number `8` is the the current mosek version). Refer to official [Installation doc](http://docs.mosek.com/7.0/toolsinstall/Linux_UNIX_installation_instructions.html).

- Qt5, OpenGL and GLUT
	- Install Qt5 (refer to [this post](https://github.com/Cockatrice/Cockatrice/wiki/Compiling-Cockatrice-(Linux)))
		- `sudo apt-get install qt5-default qttools5-dev qttools5-dev-tools` 
	- Install OpenGL and Glut librarues
		- `$ sudo apt-get install freeglut3-dev`

- update and change CXX compiler
	- add following line to the end of `~\.bashrc`:

	>     	CC=/usr/bin/gcc
	>     	export CC
	>     
	>     	CXX=/usr/bin/g++
	>     	export CXX

	- update and specify new g++ compilation
	> 	$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
	>     	$ sudo apt-get update
	>     	$ sudo apt-get install gcc-5 g++-5
	>	$ sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 1

- Follow the usual CMake procedure:

>     $ cd  Framefab
>     $ mkdir build
>     $ cd build
>     $ cmake .. (/path/to/FrameFab)
>     $ make -j4

## 3. FrameFab instructions
1. Read a .obj file into **FrameFab**.
2. Click **Choose ceiling**, choose the edges that you want them to be *ceiling*. Click again or press ESC to finish.
3. Click **Choose base**, choose the vertexes that you want them to be *base*. Click again or press ESC to finish.
4. Click **Project** to project *base vertexes* to a flat plane which is below the lowest position of all the vertexes. The vertexes projected on the plane are *fixed vertexes* and the edges connecting *base vertexes* and *fixed vertexes* are *pillars*.
5. Click **FiberPrint** to run the whole process.
6. When it is done, you can turn on *Heat* under the *Edge* mode to see the result of *layer-decomposition*.

## Common Installation problems

- ***The CXX compiler identification is unknown, no C++ compiler is found in cmake process***. Make sure you have c++ compiler on your system. Run `sudo apt-get install build-essential`, refer to [this post](http://stackoverflow.com/questions/9699930/cmake-complains-the-cxx-compiler-identification-is-unknown).

- ***Everything seems properly configured to fix previous cmake bugs, but why I still get the same error in Cmake?*** After an unsuccessful build, we must remove CMakeCache.txt (or simply clear the build directory); otherwise cmake will report the same error even if the needed package has been installed. ([refer](http://askubuntu.com/questions/374755/what-package-do-i-need-to-build-a-qt-5-cmake-application))

- ***c++: error: unrecognized command line option ‘-std=c++14’*** This happens when your current compiler (C++ in this case, or g++ version under 5.2) doesn't support -std=c++14. Look at [this post](http://stackoverflow.com/questions/32674202/cmake-make-unrecognized-command-line-option-std-c14-but-g-does) for solution. You can specify another compiler using technique addressed in [this post](http://stackoverflow.com/questions/13054451/cmake-problems-specifying-the-compiler-2). Please remember don't set compiler in CMakeLists file but set your environment variables `CC` and `CXX`. In the test machine (Ubuntu), we add following lines in **~\.bashrc**

    	CC=/usr/bin/gcc
    	export CC
    
    	CXX=/usr/bin/g++
    	export CXX

    By doing this, we change default CXX compiler from `c++` to `g++`, but default g++ compiler (version 4.8) doe not support -std=c++14. Thus, we have to install g++-5.4 (>5.2) and tell the system to refer to this new version whenever we run `g++`. Refer to [this post](https://gist.github.com/beci/2a2091f282042ed20cda), we can upgrade and set new g++ by:

    	$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    	$ sudo apt-get update
    	$ sudo apt-get install gcc-5 g++-5
		$ sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 1
	 You can check the update result by `$ g++ --version`.

- `-std=c++14` is needed for GTEngine's successful compilation.
