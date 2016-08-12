# FrameFab

## 1. Dependencies

### 1.1 General ways to include libraries in Visual Studio 2013 C++ projects
1. Download src/.lib/.dll from website.
2. **"Linker > General > Additional Library Directory"**: add "*$(LIBRARYDIR)\include*".
3. **"Linker > Input > Additional Dependencies"**: add *".lib"*.
4. **"My Computer > Properties > Advanced > Environment Variables"**:  add "*$(LIBRARYDIR)\bin*".

### 1.2 Qt
Download **Qt 5.5.1** from "*https://www.qt.io/download/*".
(OpenGL should be included in Qt.)

### 1.3 LAPACK
Download **LAPACK 3.5.0** from "*http://www.netlib.org/lapack/*".

### 1.4 Mosek
Download **Mosek 7.0** from "*https://www.mosek.com/resources/downloads*".

### 1.5 Eigen
Download **Eigen 3** from "*http://eigen.tuxfamily.org/index.php?title=Main_Page*".

### 1.6 Geometric Tools Engine
Download **Geometric Tools Engine 2.4** from "*http://www.geometrictools.com/Downloads/Downloads.html*".

## 2. FrameFab instructions
1. Read a .obj file into **FrameFab**.
2. Click **Choose ceiling**, choose the edges that you want them to be *ceiling*. Click again or press ESC to finish.
3. Click **Choose base**, choose the vertexes that you want them to be *base*. Click again or press ESC to finish.
4. Click **Project** to project *base vertexes* to a flat plane which is below the lowest position of all the vertexes. The vertexes projected on the plane are *fixed vertexes* and the edges connecting *base vertexes* and *fixed vertexes* are *pillars*.
5. Click **FiberPrint** to run the whole process.
6. When it is done, you can turn on *Heat* under the *Edge* mode to see the result of *layer-decomposition*.

