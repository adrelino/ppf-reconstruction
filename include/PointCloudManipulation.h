//
//  PointCloudManipulation.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef __PointPairFeatures__PointCloudManipulation__
#define __PointPairFeatures__PointCloudManipulation__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <unordered_map>
#include <vector>

#include "Params.h"


#include "Constants.h"
#include "CPUTimer.h"


using namespace Eigen;
using namespace std;

class PointCloud;

namespace PointCloudManipulation {

    void pointSetPCA(const vector<Vector3f>& pts, Vector3f& centroid, Vector3f& normal, float& curvature);

    float registrationErrorTra(vector< std::shared_ptr<PointCloud> >& frames);

    float registrationErrorRot(vector< std::shared_ptr<PointCloud> >& frames);


    double getPointCloudDiameter(PointCloud& m);

    //PointCloud projectPointsAndNormals(Isometry3f P, PointCloud C);

    Matrix3f covarianceOfNeighbours(const vector<Vector3f>& pts, const Vector3f p1, const float neighRadius);

    vector<Vector3f> estimateNormals(const vector<Vector3f>& pts, const vector<Vector3f>& oldNormals, const float neighRadius);

    void reestimateNormals(PointCloud &C, const float neighRadius);

    void downSample(PointCloud& C, float voxelSize);

    Translation3f getTranslationToCentroid(PointCloud& C);

    Vector3f getCentroid(vector<Vector3f>& pts);
    Vector3f getNormal(vector<Vector3f>& pts);

    int nearestNeighbourIdx(vector<Vector3f>& pts, Vector3f pt);

    float getClosesPoints(PointCloud& srcCloud, PointCloud& dstCloud, vector<Vector3f>& src, vector<Vector3f>& dst, float thresh, bool useFlann, vector<Vector3f>& nor, vector<Vector3f>& norSrc);

    //float getClosesPoints(PointCloud& srcCloud, vector<PointCloud>& dstClouds, vector<Vector3f> &src, vector<Vector3f> &dst,
    //float thresh);

    //float getClosesPoints(vector<PointCloud>& frames, int srcIndex, vector<Vector3f> &src, vector<Vector3f> &dst,
    //float thresh);

    Isometry3d leastSquaresEstimatedTrajectoryOntoGroundTruth(std::vector< std::shared_ptr<PointCloud> >& frames);
    vector<double> ateVector(std::vector< std::shared_ptr<PointCloud> >& frames, Isometry3d S);
    vector<double> rpeVector(std::vector< std::shared_ptr<PointCloud> >& frames, int delta);




}

namespace ICP {

    Isometry3f computeStepPointToPointWithScale(vector<Vector3f> &src,vector<Vector3f> &dst,bool withScale);
    Isometry3f pointToPlane(vector<Vector3f> &src,vector<Vector3f> &dst,vector<Vector3f> &nor);
    Isometry3f pointToPoint(vector<Vector3f> &src,vector<Vector3f> &dst);

}

#endif /* defined(__PointPairFeatures__PointCloudManipulation__) */




