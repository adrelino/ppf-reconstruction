//
//  PointPairFeatures.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.07.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "PointPairFeatures.h"

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include "PPF.h"
#include "math.h"

namespace PointPairFeatures{

Isometry3f getTransformationBetweenPointClouds(MatrixXf mSmall, MatrixXf sSmall){

    GlobalModelDescription map =  buildGlobalModelDescription(mSmall);

    return getTransformationBetweenPointClouds(mSmall,sSmall,map);
}

Isometry3f getTransformationBetweenPointClouds(MatrixXf mSmall, MatrixXf sSmall, GlobalModelDescription map){

    MatchesWithSceneRefIdx pair = matchSceneAgainstModel(sSmall, map);

    vector<MatrixXi> accVec = voting(pair,mSmall.rows());

    Poses Pests = computePoses(accVec, mSmall, sSmall,pair.second);

    vector<Poses> clusters = clusterPoses(Pests);

    Pests = averagePosesInClusters(clusters);

    return Pests[0].first;
}


void printBucket(Bucket v){
    cout<<v.size()<< "::::";
    for(auto i : v){
        i.print();
    }
}

void printMap(GlobalModelDescription m){
    for (auto kv : m) {
        cout << &kv.first << " : ";
        printBucket(kv.second);
        cout << endl;
    }
}


KeyBucketPairList print10(GlobalModelDescription &mymap) {
    KeyBucketPairList myvec(mymap.begin(), mymap.end());
    assert(myvec.size() >= 10);
    //std::partial_sort(myvec.begin(), myvec.begin() + 10, myvec.end(),
    std::sort(myvec.begin(), myvec.end(),
                      [](const KeyBucketPair &lhs, const KeyBucketPair &rhs) {
                          return lhs.second.size() > rhs.second.size();
                      });
    
    for (int i = 0; i < 10; ++i) {
        cout<<"the 10 largest buckets are:"<<endl;
        cout<<"size="<<myvec[i].second.size()<<endl;//<<" key= "<<myvec[i].first;
        //printBucket(myvec[i].second);
        cout<<endl;
        //std::cout << i << ": " << myvec[i].first << "-> " << myvec[i].second << "\n";
    }
    
    return myvec;
}


GlobalModelDescription buildGlobalModelDescription(MatrixXf m){
    int Nm=m.rows();
    cout<<"PointPairFeatures::buildGlobalModelDescription from "<<Nm<<" pts"<<endl;

    
    RowVectorXf p1(6),p2(6);

    
    GlobalModelDescription map;
    
    
    int numPPFs=0;
    
    for (int i=0; i<Nm; i++) {
        p1=m.row(i);
        for (int j=0; j<Nm; j++) {
            if(i==j) continue;
            p2=m.row(j);
            
            //p1 << 0,0,0,  1,0,0;
            //p2 << 1,1,0,  0,1,0;
            
            //std::cout << p1 << std::endl;
            //std::cout << p2 << std::endl;
            numPPFs++;
            
            PPF ppf=PPF::makePPF(p1,p2,i,j);
            
            map[ppf].push_back(ppf);
        }
    }
    
//    vector<float> numb;
    
//    for (auto it : map){
//        float x=it.second.size();
//        numb.push_back(x);
//    }
    
    //cout<<"PPF's discretisation values: ddist="<<ddist<<" dangle"<<dangle<<endl;
    cout<<"PointPairFeatures::buildGlobalModelDescription from "<<Nm<<" pts, yielding "<<numPPFs<< " PPF's hashed into "<<map.size()<<" Buckets: "<<endl;
    //LoadingSaving::summary(numb);
    //LoadingSaving::saveVector("buckets.txt", numb);
    
    return map;
    
}

std::pair<Matches, vector<int> > matchSceneAgainstModel(MatrixXf s, GlobalModelDescription model){
    long Sm=s.rows(); //number of model sample points
    int numberOfSceneRefPts=sceneRefPtsFraction*Sm;
    cout<<"PointPairFeatures::matchSceneAgainstModel with "<<numberOfSceneRefPts<< " sceneRefPts"<<endl;

    Matches matches;

    RowVectorXf p1(6),p2(6);

    vector<int> sceneIndexToI;

    for (int index=0; index<numberOfSceneRefPts; index++) {
        
        int i=rand() % Sm;  //TODO: dont pick at random, but equally spaced
        //i=index;//Testing
        p1=s.row(i);
        sceneIndexToI.push_back(i);
        
        for (int j=0; j<s.rows(); j++) {
            if(i==j) continue;
            p2=s.row(j);
                        
            PPF ppf = PPF::makePPF(p1,p2,i,j);
            ppf.index=index;
            
            auto it = model.find(ppf);
            
            if(it != model.end()){
                Bucket modelBucket = it->second;
                Match match = {ppf,modelBucket};
                matches.push_back(match);
            }else{
                //cout<<"no match index="<<index<<"  j="<<j<<endl;
            }
        }
    }
    
//    vector<float> numb;
    
//    for (auto it : matches){
//        float x=it.modelPPFs.size();
//        //cout<<it.scenePPF.i<<" "<<it.scenePPF.j<<" model bucket size="<<x<<endl;
//        numb.push_back(x);
//    }

    cout<<"PointPairFeatures::matchSceneAgainstModel with "<<numberOfSceneRefPts<< " sceneRefPts to "<<model.size()<<" model buckets"<<endl;

    //LoadingSaving::summary(numb);
    //LoadingSaving::saveVector("buckets_matched.txt", numb);

    return make_pair(matches,sceneIndexToI);

}


vector<MatrixXi> voting(MatchesWithSceneRefIdx matches, int Nm){
    vector<MatrixXi> accVec;
    int numberOfSceneRefPts=matches.second.size();
    for (int i=0; i<numberOfSceneRefPts; i++) {
        MatrixXi acc=MatrixXi::Zero(Nm,nangle);
        accVec.push_back(acc);
    }

    
    cout<<"PointPairFeatures::voting for ACC "<<numberOfSceneRefPts<<" sceneRefPts * "<<accVec[0].rows()<<" * "<<accVec[0].cols()<<endl;
    
    
    for (auto it : matches.first){
        int sr=it.scenePPF.index;
        if(std::isnan(it.scenePPF.alpha)){
            cout<<sr<<" sr isnan"<<endl;
            continue;
        }
        //MatrixXi acc=accVec[sr];
        for (auto it1:it.modelPPFs){
            long mr=it1.i;
            if(std::isnan(it1.alpha)){
                cout<<mr<<" mr isnan"<<endl;
                continue;
            }

            float alpha=getAngleDiffMod2Pi(it1.alpha,it.scenePPF.alpha);


            int alphaDiscretised=alpha/dangle;

            long r=accVec[sr].rows();
            long c=accVec[sr].cols();
            if(mr<0 || mr>=r || alphaDiscretised>=c || alphaDiscretised <0){

                cout<<"alpha: "<<(alpha)<<endl;
                cout<<"alpha dangle"<<dangle<<endl;

                cout<<"alpha discretised"<<alphaDiscretised<<endl;
                cout<<"accSize="<<accVec[sr].rows() << " " << accVec[sr].cols()<<endl;

                cout<<mr<<"*"<<alphaDiscretised<<endl;
            }


            accVec[sr](mr,alphaDiscretised)=accVec[sr](mr,alphaDiscretised)+1;
        }
        //cout<<acc<<endl;
    }
    cout<<"PointPairFeatures::voting done "<<endl;

    return accVec;
}

float getAngleDiffMod2Pi(float modelAlpha, float sceneAlpha){
    float alpha = sceneAlpha - modelAlpha; //correct direction

    //cout<<"modelAlpha: "<<degrees(modelAlpha)<<endl;
    //cout<<"sceneAlpha: "<<degrees(sceneAlpha)<<endl;
    //cout<<"alpha: "<<degrees(alpha)<<endl;

    while(alpha<0.0) alpha += M_PI*2;
    alpha=fmod(alpha,M_PI*2.0f);

    //now alpha is in interval 0 -> 2*pi

    return alpha;
}

Poses computePoses(vector<MatrixXi> accVec, MatrixXf m, MatrixXf s,vector<int> sceneIndexToI){
    cout<<"PointPairFeatures::computePoses"<<endl;

    Poses vec;

    for (int index=0; index<accVec.size(); index++) {
        MatrixXi acc=accVec[index];

        int sr=sceneIndexToI[index];
        int mr;
        int alphaD;
        int score=acc.maxCoeff(&mr, &alphaD); //TODO detect multiple peaks, but ask betram if this only happens if there are multiple object instances in the scene


        float alpha=alphaD*dangle;

        //ref points (just one, not both of the ppf)
        RowVectorXf modelRefPt = m.row(mr);
        RowVectorXf sceneRefPt = s.row(sr);

        Isometry3f P = alignSceneToModel(sceneRefPt,modelRefPt,alpha);

        vec.push_back(std::make_pair(P,score));
    }
    
    return vec;
}

Isometry3f alignSceneToModel(RowVectorXf q1, RowVectorXf p1, float alpha){
    //Isometry3f Tgs(ppfScene.T.inverse()); //TODO: check if it makes sense to store and reuse T from ppf's
    Isometry3f Tgs = PPF::twistToLocalCoords(q1.head(3),q1.tail(3)).inverse();

    AngleAxisf Rx(alpha, Vector3f::UnitX());

    //Isometry3f Tmg(ppfModel.T);
    Isometry3f Tmg = PPF::twistToLocalCoords(p1.head(3),p1.tail(3));

    Isometry3f Pest = Tgs*Rx*Tmg;

    return Pest;
}

//returns true if farthest neighbors in cluster fit within threshold
//http://en.wikipedia.org/wiki/Complete-linkage_clustering
bool isClusterSimilar(Poses cluster1, Poses cluster2){
    for(auto pose2 : cluster2){
        bool isSimilar = std::all_of(cluster1.begin(), cluster1.end(), [&](Pose pose1){return isPoseSimilar(pose1.first, pose2.first);});
        if(!isSimilar) return false;
    }

    return true;
}

bool isPoseSimilar(Isometry3f P1, Isometry3f P2){
    Vector3f    tra1 = P1.translation();
    Quaternionf rot1(P1.linear());

    Vector3f    tra2 = P2.translation();
    Quaternionf rot2(P2.linear());



    //Translation
    float diff_tra=(tra1-tra2).norm();
    //Rotation
    float d = rot1.dot(rot2);
    //float diff_rot= 1 - d*d; //http://www.ogre3d.org/forums/viewtopic.php?f=10&t=79923

    //    rot1.angularDistance(rot2);

    //http://math.stackexchange.com/questions/90081/quaternion-distance
    //float thresh_rot=0.25; //0same, 1 180deg ////M_PI/10.0; //180/15 = 12
    //float diff_rot_bertram = acos((rot1.inverse() * rot2).norm()); //bertram

    float diff_rot_degrees = rad2deg(acos(2*d - 1));

    //cout<<"diff_rot_0to1nor\t="<<diff_rot<<endl;
    //cout<<"diff_rot_bertram\t="<<diff_rot_bertram<<endl;

    //cout<<std::fixed<<std::setprecision(3);

    //cout<<"rot="<<diff_rot_degrees<<"<="<<thresh_rot_degrees<<" && tra="<<diff_tra<<"<= "<<thresh_tra<<" ?: ";

    if(diff_rot_degrees <= thresh_rot_degrees && diff_tra <= thresh_tra){
      //  cout<<"yes"<<endl;
        return true;
    }
    //cout<<"no"<<endl;
    return false;
}

bool isPoseCloseToIdentity(Isometry3f P1, float eps){
    Vector3f    tra1 = P1.translation();
    Quaternionf rot1(P1.linear());

    //Translation
    float diff_tra=tra1.norm();
    //Rotation
    Vector3f rot1V(rot1.x(),rot1.y(),rot1.z());  //w should be close to 1
    float diff_rot=rot1V.norm();

    return diff_tra < eps && diff_rot < eps;

}

void printPose(Pose pose,string title){
    if(title!="") title+=" ";
    cout<< title <<"score : "<<pose.second<<endl;
    printPose(pose.first);
}


void printPose(Isometry3f P,string title){
    //cout<<P.matrix()<<endl;
    Vector3f tra(P.translation());
    Quaternionf q(P.linear());
    Vector4f rot(q.x(),q.y(),q.z(),q.w());
    //cout<<setprecision(3);
    if(title!="") cout<<title<<endl;
    cout<<"tra: "<<tra.transpose()<<endl;
    cout<<"rot: "<<rot.transpose()<<endl;
}

void printPoses(Poses vec){
    for(auto pose : vec){
        printPose(pose);
    }
}

vector<Poses> clusterPoses (Poses vec){

   vec=sortPoses(vec);

   vector< Poses > clusters;
    
    for(auto pose : vec){
        Poses cluster;
        cluster.push_back(pose); //initially, each cluster contains just one pose;
        clusters.push_back(cluster);
    }

    int i=0;
    int n=clusters.size();

    for(int i=0; i<n; n=clusters.size(),i++){
        for(int j=0; j<n; n=clusters.size(),j++){
            if(i==j) continue;
            //cout<<"Cluster1 "<<i<<"\\"<<n-1<<endl;
            Poses cluster1=clusters[i];
            //cout<<"Cluster2 "<<j<<"\\"<<n-1<<endl;
            Poses cluster2=clusters[j];
            //cout<<"size before merge:"<<cluster1.size()<<","<<cluster2.size()<<endl;

            if(isClusterSimilar(cluster1,cluster2)){
                cluster1.insert(cluster1.end(),cluster2.begin(),cluster2.end());
                clusters.erase(clusters.begin() + j);
            }
            //cout<<"size after merge:"<<cluster1.size()<<","<<cluster2.size()<<endl;
            clusters[i]=cluster1;
        }
    }

    cout<<"Produced "<<clusters.size()<<" clusters with each #poses:"<<endl;
    for(auto cluster : clusters){
        cout<<cluster.size()<<endl;
        printPoses(cluster);
    }

    
    return clusters;
}

Pose averagePosesInCluster(Poses cluster){
    //cout<<cluster.size()<<endl;
    Vector3f tra(0,0,0);
    Vector4f rot(0,0,0,0); //w,x,y,z
    int votes=0;
    for(Pose pose : cluster){
        tra += pose.first.translation(); //TODO: maybe weight using number of votes?
        Quaternionf q = Quaternionf(pose.first.linear());
        rot += Vector4f(q.x(),q.y(),q.z(),q.w());  //w last http://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#ad90ae48f7378bb94dfbc6436e3a66aa2
        votes += pose.second;
    }
    tra /= cluster.size();
    //my stackexchange posts:
    //http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions/
    //http://math.stackexchange.com/questions/61146/averaging-quaternions/

    //mean is a good approx of quaternion interpolation:
    // http://www.mathworks.com/matlabcentral/fileexchange/40098-averaging-quaternions
    // http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
    // http://www.soest.hawaii.edu/wessel/courses/gg711/pdf/Gramkow_2001_JMIV.pdf
    // http://objectmix.com/graphics/132645-averaging-quaternions-2.html
    rot /= cluster.size();

    Isometry3f P = Translation3f(tra)*Quaternionf(rot);

    return std::make_pair(P,votes);

}

Vector4f avg_quaternion_markley(MatrixXf Q){
    Matrix4f A = Matrix4f::Zero();
    int M = Q.rows();

    for(int i=0; i<M; i++){
        Vector4f q = Q.row(i);
        A = q*q.adjoint() + A;
    }

    A=(1.0/M)*A;


    SelfAdjointEigenSolver<MatrixXf> eig(A);
//    cout<<"A"<<endl<<A<<endl;
//    cout<<"vecs"<<endl<<eig.eigenvectors()<<endl;
//    cout<<"vals"<<endl<<eig.eigenvalues()<<endl;
    Vector4f qavg=eig.eigenvectors().col(3);
    return qavg;
}



Vector4f avg_quaternion_markley(Poses cluster){

    int M = cluster.size();
    MatrixXf Q(M,4);

    for(int i=0; i<M; i++){
        Pose p = cluster[i];
        Quaternionf q = Quaternionf(p.first.linear());
        RowVector4f rot = RowVector4f(q.x(),q.y(),q.z(),q.w());
        Q.row(i)=rot;
    }


    return avg_quaternion_markley(Q);
}

Quaternionf avg_quaternion_markleyQ(Poses cluster){
    Vector4f q=avg_quaternion_markley(cluster);
    return Quaternionf(q(0),q(1),q(2),q(3));
}


Poses sortPoses(Poses vec){
    //cout<<"clusterPoses"<<endl;
    //printPoses(vec);
    std::sort(vec.begin(), vec.end(), [](const Pose & a, const Pose & b) -> bool{ return a.second > b.second; });
    //cout<<"sorted"<<endl;
    //printPoses(vec);
    return vec;
}


Poses averagePosesInClusters(vector<Poses> clusters){
    Poses vec;

    for(auto cluster : clusters){
        vec.push_back(averagePosesInCluster(cluster));
    }

    vec=sortPoses(vec);



    return vec;
}

void err(Isometry3f P, Pose PoseEst){
    cout<<"//------- Error between P_gold and P_est with "<<PoseEst.second<<" votes --------\\"<<endl;
    Isometry3f Pest=PoseEst.first;
    err(P,Pest);
}

void err(Isometry3f P, Isometry3f Pest){
    Vector3f tra(P.translation());
    Quaternionf q(P.linear());
    Vector4f rot(q.x(),q.y(),q.z(),q.w());

    Vector3f tra_est(Pest.translation());
    Quaternionf qest(Pest.linear());
    Vector4f rot_est(qest.x(),qest.y(),qest.z(),qest.w());

    Vector3f tra_diff = (tra - tra_est);
    float tra_error=tra_diff.array().cwiseAbs().sum();

    Vector4f rot_diff = (rot - rot_est);
    float rot_error=rot_diff.array().cwiseAbs().sum();

//    if(rot_error>1.5){
//        cout<<"flipping rot"<<endl;
//        rot_est*=-1;
//        rot_diff = (rot - rot_est);
//        rot_error=rot_diff.array().cwiseAbs().sum();
//    }


    //cout<<setprecision(3);
    cout<<"tra_ori: "<<tra.transpose()<<endl;
    cout<<"tra_est: "<<tra_est.transpose()<<endl;
    //cout<<"tra_dif: "<<tra_diff.transpose()<<"\n --->tra_error: "<<tra_error<<endl;
    cout<<"tra_error:----------------------------------------------> "<<tra_error<<endl;

    cout<<"rot_ori: "<<rot.transpose()<<endl;
    cout<<"rot_est: "<<rot_est.transpose()<<endl;
    //cout<<"rot_dif: "<<rot_diff.transpose()<<endl;
    cout<<"rot_error:----------------------------------------------> "<<rot_error<<endl;

    //cout<<"P: "<<endl<<P.matrix()<<endl;
    //cout<<"P_est: "<<endl<<Pest.matrix()<<endl;
}

} //end namespace
