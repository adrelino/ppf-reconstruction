ppf-reconstruction
===================

3D Object Reconstruction using Point Pair Features aims at reconstructing 3D objects by robustly registering multiple scans of an object from different viewpoints.

An initial alignment between any two overlapping point clouds is obtained via a voting scheme which matches similar point pair features and thus constrains the 6dof rigid body motion between two frames. This initial estimate is then refined using standard point-to-plane or point-to-point ICP.

In a subsequent global optimization step, we build up a pose graph of the initial pose estimates.
A Vertex is connected to its k-nearest-neighbours, where we either use the distances of the translational part of the SE(3) camera poses or their percentual point cloud overlap. Edges are added for all the nearest neighbours between the point clouds belonging to a vertex which distances fall below a certain threshold. This graph is fed into the g2o framework, which minimizes the global geometric point-to-plane distance between all pairs of connected vertices using Levenberg-Marquardt Generalized-ICP.

This refined registration of all the scans used may now be integrated and their corresponding point clouds fused and then meshed to obtain the final reconstructed 3D object mesh.

### References
Drost, B., Ulrich, M., Navab, N., & Ilic, S. (2010). [Model globally, match locally: Efficient and robust 3D object recognition](http://campar.cs.tum.edu/pub/drost2010CVPR/drost2010CVPR.pdf). In Computer Vision and Pattern Recognition (CVPR), 2010 IEEE Conference on (pp. 998-1005). IEEE

Zhang, Z. (1994). Iterative point matching for registration of free-form curves and surfaces. International journal of computer vision, 13(2), 119-152.

Kummerle, R., Grisetti, G., Strasdat, H., Konolige, K., & Burgard, W. (2011, May). g 2 o: A general framework for graph optimization. In Robotics and Automation (ICRA), 2011 IEEE International Conference on (pp. 3607-3613). IEEE.


### Dependencies
Eigen3, OpenCV, OpenGL, FreeGLUT, C++11, g2o
