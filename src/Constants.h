//
//  Params.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef PointPairFeatures_Params_h
#define PointPairFeatures_Params_h

#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_map>
#include "PPF.h"

using namespace std;

//Typedefs
//
typedef vector<PPF> Bucket;
//key type, value, hasher
typedef unordered_map<int,Bucket> GlobalModelDescription;
struct Match {PPF scenePPF; Bucket modelPPFs;};
typedef std::pair<int, Bucket> KeyBucketPair;
typedef vector<KeyBucketPair> KeyBucketPairList;
typedef vector<Match> Matches;
typedef pair<Isometry3f,int> Pose;
typedef vector<Pose> Poses;
typedef std::pair< Matches,vector<int> > MatchesWithSceneRefIdx;



struct TrainedModel{GlobalModelDescription modelDescr; PointCloud mSmall; Translation3f centroid;};

//Params
//

//model diameter, downsampling
static float diamM=0.15f; //model diameter //stanford bunny // max dist is diagonal
static float tau_d=0.1f; //sampling rate, set like in paper. ddist=tau_d*diam(M);
static float ddist = tau_d*diamM;//0.01 paper: 0.05*diam(M)=o.o5*0.15=0.0075

// reestimate normals after downsampling
static float neighbourBallSize=ddist*2.5f; //neighbors in 2*ddist ball around p1 (e.g. 2cm)

//ppf's
static int ndist=1/tau_d; //number of distance buckets
static int nangle=30;     //number of angle buckets
static float dangle = 2*M_PI/nangle; //normal's derivation of up to 12 degree like in paper (360/30=12)
static float sceneRefPtsFraction = 0.3f; //20percent of pts in scene picked (at random so far) as reference points to compute ppfs to all other model points

//for pose cluster averaging
static float thresh_tra = 0.05f * diamM; //float thresh_tra=0.02; //2cm
static float thresh_rot_degrees = 20; //180 max

//Macros
//
#define rad2deg(r) (180*(r)/M_PI)
#define deg2rad(d) (M_PI*(d)/180)

namespace Colormap{


//http://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
static float interpolate( float val, float y0, float x0, float y1, float x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

static float base( float val ) {
    if ( val <= -0.75 ) return 0;
    else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 ) return 1.0;
    else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

static float red( float gray ) {
    return base( gray - 0.5 );
}
static float green( float gray ) {
    return base( gray );
}
static float blue( float gray ) {
    return base( gray + 0.5 );
}

static RowVector3f Jet(float gray){
    return RowVector3f(red(gray),green(gray),blue(gray));
}

}


#endif
