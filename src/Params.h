//
//  Params.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef PointPairFeatures_Params_h
#define PointPairFeatures_Params_h

struct PPFIndexPair { int i; int j; PPF ppf;};
typedef vector<PPFIndexPair> Bucket;
typedef unordered_map<PPF,Bucket,PPF> GlobalModelDescription;
struct Match {PPFIndexPair scenePPF; Bucket modelPPFs;};

typedef std::pair<PPF, Bucket> KeyBucketPair;
typedef vector<KeyBucketPair> KeyBucketPairList;
typedef vector<Match> Matches;

static constexpr double diamM=0.15; //stanford bunny
static constexpr double tau_d=0.05; //sampling rate, set like in paper. ddist=tau_d*diam(M);
static constexpr double ddist = tau_d*diamM;//0.01 paper: 0.05*diam(M)=o.o5*0.15=0.0075 // max dist is diagonal

static const int ndist=1/tau_d;
static const int nangle=30;
static constexpr double dangle = M_PI/nangle; //normal or. uf up to 12* like in paper (360/30=12)


#endif
