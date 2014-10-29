//
//  Params.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef PointPairFeatures_Params_h
#define PointPairFeatures_Params_h

#include "PPF.h"
#include <vector>
#include <unordered_map>
#include <vector>

using namespace std;

//Typedefs
//
typedef vector<PPF> Bucket;
typedef unordered_map<PPF,Bucket,PPF> GlobalModelDescription;
struct Match {PPF scenePPF; Bucket modelPPFs;};
typedef std::pair<PPF, Bucket> KeyBucketPair;
typedef vector<KeyBucketPair> KeyBucketPairList;
typedef vector<Match> Matches;
typedef pair<Projective3d,int> Pose;
typedef vector<Pose> Poses;
typedef std::pair< Matches,vector<int> > MatchesWithSceneRefIdx;

//Params
//

//model diameter, downsampling
static double diamM=0.20; //model diameter //stanford bunny // max dist is diagonal
static double tau_d=0.075; //sampling rate, set like in paper. ddist=tau_d*diam(M);
static double ddist = tau_d*diamM;//0.01 paper: 0.05*diam(M)=o.o5*0.15=0.0075

// reestimate normals after downsampling
static double neighbourBallSize=ddist*2.5; //neighbors in 2*ddist ball around p1 (e.g. 2cm)

//ppf's
static int ndist=1/tau_d; //number of distance buckets
static int nangle=30;     //number of angle buckets
static double dangle = 2*M_PI/nangle; //normal's derivation of up to 12 degree like in paper (360/30=12)
static double sceneRefPtsFraction = 0.4; //20percent of pts in scene picked (at random so far) as reference points to compute ppfs to all other model points

//for pose cluster averaging
static double thresh_tra = 0.05 * diamM; //double thresh_tra=0.02; //2cm
static double thresh_rot_degrees = 10; //180 max

//Macros
//
#define rad2deg(r) (180*(r)/M_PI)
#define deg2rad(d) (M_PI*(d)/180)

namespace Colormap{


//http://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
static double interpolate( double val, double y0, double x0, double y1, double x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

static double base( double val ) {
    if ( val <= -0.75 ) return 0;
    else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 ) return 1.0;
    else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

static double red( double gray ) {
    return base( gray - 0.5 );
}
static double green( double gray ) {
    return base( gray );
}
static double blue( double gray ) {
    return base( gray + 0.5 );
}

static RowVector3f Jet(double gray){
    return RowVector3f(red(gray),green(gray),blue(gray));
}

}


#endif
