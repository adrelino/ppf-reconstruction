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

typedef vector<PPF> Bucket;
typedef unordered_map<PPF,Bucket,PPF> GlobalModelDescription;
struct Match {PPF scenePPF; Bucket modelPPFs;};

typedef std::pair<PPF, Bucket> KeyBucketPair;
typedef vector<KeyBucketPair> KeyBucketPairList;
typedef vector<Match> Matches;
typedef pair<Projective3d,int> Pose;
typedef vector<Pose> Poses;

static const double diamM=0.20; //stanford bunny
static const double tau_d=0.075; //sampling rate, set like in paper. ddist=tau_d*diam(M);
static const double ddist = tau_d*diamM;//0.01 paper: 0.05*diam(M)=o.o5*0.15=0.0075 // max dist is diagonal

static const int ndist=1/tau_d;
static const int nangle=30;
static const double dangle = 2*M_PI/nangle; //normal or. uf up to 12* like in paper (360/30=12)

#define degrees(r) (180*(r)/M_PI)
#define radians(d) (M_PI*(d)/180)


#endif
