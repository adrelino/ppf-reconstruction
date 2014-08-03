//
//  test2.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 27.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//
#include "Visualize.h"
#include "LoadingSaving.h"
#include "PointPairFeatures.h"


int main(int argc, char * argv[])
{
    
    MatrixXd mSmall=LoadingSaving::loadXYZ("bunny/model_downSampled.xyz");
    MatrixXd sSmall=LoadingSaving::loadXYZ("bunny/scene_downSampled.xyz");
    
    Visualize* inst=Visualize::getInstance();
    
    inst->model=mSmall;
    inst->scene=sSmall;
    
    Visualize::visualize();
}