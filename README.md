ppf-reconstruction
===================

3D Object Reconstruction using Point Pair Features aims at reconstructing 3D objects by robustly registering multiple scans of an object from different viewpoints.

An initial alignment between any two overlapping point clouds is obtained via a voting scheme which matches similar point pair features and thus constrains the 6dof rigid body motion between two frames. This initial estimate is then refined using ICP.

In a subsequent global optimization step, we build up a pose graph of the initial pose estimates based on the pose distances in the SE(3) group and their point cloud overlap. Then, we try to optimize them all together, again using ICP, to get a coherent registration of all the scans used.

Finally, the registered scans can be integrated and their corresponding point clouds fused to obtain the final reconstructed 3D object point cloud.

### References
Drost, B., Ulrich, M., Navab, N., & Ilic, S. (2010). [Model globally, match locally: Efficient and robust 3D object recognition](http://campar.cs.tum.edu/pub/drost2010CVPR/drost2010CVPR.pdf). In Computer Vision and Pattern Recognition (CVPR), 2010 IEEE Conference on (pp. 998-1005). IEEE

Zhang, Z. (1994). Iterative point matching for registration of free-form curves and surfaces. International journal of computer vision, 13(2), 119-152.


### Dependencies
Eigen3, OpenCV, OpenGL, FreeGLUT, C++11
