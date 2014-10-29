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

//Params
//

//model diameter, downsampling
static const double diamM=0.20; //model diameter //stanford bunny // max dist is diagonal
static const double tau_d=0.075; //sampling rate, set like in paper. ddist=tau_d*diam(M);
static const double ddist = tau_d*diamM;//0.01 paper: 0.05*diam(M)=o.o5*0.15=0.0075

// reestimate normals after downsampling
static const double neighbourBallSize=ddist*2; //neighbors in 2*ddist ball around p1 (e.g. 2cm)

//ppf's
static const int ndist=1/tau_d; //number of distance buckets
static const int nangle=30;     //number of angle buckets
static const double dangle = 2*M_PI/nangle; //normal's derivation of up to 12 degree like in paper (360/30=12)
static const double sceneRefPtsFraction = 0.2; //20percent of pts in scene picked (at random so far) as reference points to compute ppfs to all other model points

//for pose cluster averaging
static const double thresh_tra = 0.05 * diamM; //double thresh_tra=0.02; //2cm
static const double thresh_rot_degrees = 30; //180 max

//Macros
//
#define rad2deg(r) (180*(r)/M_PI)
#define deg2rad(d) (M_PI*(d)/180)


#endif
