ppf-reconstruction
===================

This work aims at reconstructing 3D objects by robustly and accurately registering multiple range images of an object from different viewpoints.

An initial alignment between any two overlapping scans is obtained via a voting scheme which matches similar point pair features and thus constrains the relative 6DoF rigid body motion between the poses of two viewpoints. This initial alignment is then refined using pairwise point-to-plane ICP.

The result of this step is a tree of relative pose constraints. In a subsequent global optimization step, we build up a graph of absolute poses, our vertices, from the tree of initial relative pose estimates by adding further edges. We add edges for the k-nearest-neighbors of a vertex, taking the translational difference of the corresponding poses as a distance measure. Constraints between two vertices are added for each closest point correspondence in their respective point clouds. The global point-toplane energy is then minimized iteratively using the nonlinear least-squares method called Multiview Levenberg-Marquardt ICP.

This refined registration of all the scans used may now be integrated and their corresponding point clouds fused and then meshed to obtain the final reconstructed 3D object mesh.

### References
* Haarbach, Adrian [3D Object Reconstruction using Point Pair Features](http://adrian-haarbach.de/bscthesis_adrian.pdf). Bachelor's thesis, Technical University Munich, 2015.

* Drost, B., Ulrich, M., Navab, N., & Ilic, S. (2010). [Model globally, match locally: Efficient and robust 3D object recognition](http://campar.cs.tum.edu/pub/drost2010CVPR/drost2010CVPR.pdf). In Computer Vision and Pattern Recognition (CVPR), 2010 IEEE Conference on (pp. 998-1005). IEEE

* Chen, Y., Medioni, G (1991). Object modeling by registration of multiple range images. In Robotics and Automation, 1991. Proceedings., 1991 IEEE International Conference on, pages 2724–2729. IEEE, 1991.

* Fantoni, S., Castellani, U., & Fusiello, A. (2012, October). Accurate and Automatic Alignment of Range Surfaces. In 3DIMPVT (pp. 73-80). ISO 690	

* Kummerle, R., Grisetti, G., Strasdat, H., Konolige, K., & Burgard, W. (2011, May). g 2 o: A general framework for graph optimization. In Robotics and Automation (ICRA), 2011 IEEE International Conference on (pp. 3607-3613). IEEE.


### Dependencies

CMake, Eigen3, OpenCV, OpenGL, FreeGLUT, C++11, g2o

##### install on Mac OSX Yosemite using homebrew

make sure brew is installed correctly
```sh
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
brew doctor
```

```sh
brew install cmake
brew install eigen
```
opencv is needed to load the range images
```sh
brew tap homebrew/science
brew install opencv
```

We need xquartz for freeglut to run. For that we first need to accept a license:
```sh
sudo xcodebuild -license   //accept license
```
then we can install xquartz on the command line using cast (or alternatively downlaod and install from http://xquartz.macosforge.org/)
```sh
brew install Caskroom/cask/xquartz
```

Freeglut is needed for the visualizer. We might need a restart if a "cannot open display error" appears later on when running our code.
```sh
brew tap homebrew/x11
brew install freeglut 
```

ready to build and install g2o
```sh
cd ~/projects
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake ..
make
sudo make install
```

##### install on Linux Ubuntu 14.04

```sh
sudo apt-get install cmake libeigen3-dev opencv freeglut 
```
###### Build and install g2o

* if you have ROS indigo installed (http://wiki.ros.org/indigo/Installation/Ubuntu), there is a package for g2o:
```sh
sudo apt-get install ros-indigo-libg2o
```

* otherwise manually download and build and install g2o:
```sh
cd ~/projects
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake ..
make
sudo make install
```

### Build and run ppf-reconstruction

Now we are ready to build our code
```sh
cd ~/projects
git clone https://github.com/adrelino/ppf-reconstruction.git
cd ppf-reconstruction
mkdir build && cd build
cmake ..
make
```

##### Run with sample bunny sequences
Real dataset aquired with a PrimeSense sensor:
```sh
./ppf-reconstruction
```

Synthetic dataset rendered using Blender:
```sh
./ppf_reconstruction -dir ../samples/Bunny_Sphere/ -knn 5 -nFrames 5
```
