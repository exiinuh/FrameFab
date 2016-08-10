# FrameFab
## 1. Environment
- **CPU**: i7-3770k @ 3.50GHz
- **RAM**: 16GB
- **OS**: Windows 7-64bit
- **Compiler**: Visual Studio 2013
- **Language**: C++/C
## 2. Dependencies
### 2.1 General ways to include libraries in Visual Studio 2013 C++ projects
1. Download src/.lib/.dll from website.
2. **"Linker > General > Additional Library Directory"**: add "*$(LIBRARYDIR)\include*".
3. **"Linker > Input > Additional Dependencies"**: add *".lib"*.
4. **"My Computer > Properties > Advanced > Environment Variables"**:  add "*$(LIBRARYDIR)\bin*".
### 2.2 Qt
Download **Qt 5.5.1** from "*https://www.qt.io/download/*".
(OpenGL should be included in Qt.)
### 2.3 LAPACK
Download **LAPACK 3.5.0** from "*http://www.netlib.org/lapack/*".
### 2.4 Mosek
Download **Mosek 7.0** from "*https://www.mosek.com/resources/downloads*".
### 2.5 Eigen
Download **Eigen 3** from "*http://eigen.tuxfamily.org/index.php?title=Main_Page*".
### 2.6 Geometric Tools Engine
Download **Geometric Tools Engine 2.4** from "*http://www.geometrictools.com/Downloads/Downloads.html*".
## 3. .PWF file format
A **.PWF file format** is a mesh format used for the storage of object information in **FrameFab**. While .obj files are compatible with FrameFab, .pwf files are actually the extended version of .obj files that containing extra information from FrameFab results.

Take a .pwf file as an example, which is a result of *layer-decomposition*.
```C++
/* vertex x y z*/
v 122.966980 40.710518 10.327159
v 122.966980 40.710518 61.962955
v 84.055855 3.264418 0.000000
v 146.556137 -7.867645 0.000000
v 176.457001 48.133659 0.000000
v 132.436478 93.876434 0.000000
v 75.329422 66.145714 0.000000
v 84.055855 3.264418 -10.000000
v 146.556137 -7.867645 -10.000000
v 176.457001 48.133659 -10.000000
v 132.436478 93.876434 -10.000000
v 75.329422 66.145714 -10.000000

/* line vertex1 vertex2*/
l 1 2
l 1 3
l 1 4
l 1 5
l 1 6
l 1 7
l 3 4
l 4 5
l 5 6
l 6 7
l 7 3
l 2 3
l 2 4
l 2 5
l 2 6
l 2 7

/* pillar base-vertex fixed-vertex */
p 3 8
p 4 9
p 5 10
p 6 11
p 7 12

/* ceiling vertex1 vertex2*/
c 2 3
c 2 4
c 2 5
c 2 6
c 2 7

/* cut vertex1 vertex2 layer*/
g 1 2 1
g 1 3 1
g 1 4 1
g 1 5 1
g 1 6 1
g 1 7 1
g 3 4 2
g 4 5 2
g 5 6 2
g 6 7 2
g 7 3 2
g 2 3 2
g 2 4 2
g 2 5 2
g 2 6 2
g 2 7 2
g 3 8 1
g 4 9 1
g 5 10 1
g 6 11 1
g 7 12 1
```
## 4. FrameFab instructions
1. Read a .obj file into **FrameFab**.
2. Click **Choose ceiling**, choose the edges that you want them to be *ceiling*. Click again or press ESC to finish.
3. Click **Choose base**, choose the vertexes that you want them to be *base*. Click again or press ESC to finish.
4. Click **Project** to project *base vertexes* to a flat plane which is below the lowest position of all the vertexes. The vertexes projected on the plane are *fixed vertexes* and the edges connecting *base vertexes* and *fixed vertexes* are *pillars*.
5. Click **FiberPrint** to run the whole process.
6. When it is done, you can turn on *Heat* under the *Edge* mode to see the result of *layer-decomposition*.

