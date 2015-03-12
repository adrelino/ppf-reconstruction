
// ppf_reconstruction
//
//  Created by Adrian Haarbach on 10.03.15.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//
#include "Visualize.h"
#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include <iostream>
#include <string>
#include "Constants.h"

#include <iomanip>
#include <cmath>
#include <numeric>

#include "ApproachComponents.h"

#include <opencv2/highgui.hpp>

#include <fstream>

using namespace std;

typedef Matrix<double,6,1> Vector6d;

Vector6d summary(std::vector<double> v){
    int N = v.size();

    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / N;

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double std = std::sqrt(sq_sum / v.size() - mean * mean);

    std::sort(v.begin(),v.end());
    double min=v[0];
   // float firstQuantile=v[v.size()*0.25];
    double median= N % 2 == 1 ? v[N*0.5] : v[floor(N*0.5)]*0.5 + v[ceil(N*0.5)]*0.5;
    //float thirdQuantile=v[v.size()*0.75];
    double max=v[v.size()-1];

//    cout<<"Summary of "<<v.size()<<" bucket sizes:"<<endl;
//    cout<<"Min\t.25\tMed\tMean\t.75\tMax \tStd"<<endl;
//    cout<<min<<" \t"<<firstQuantile<<" \t"<<median<<" \t"<<round(mean*100)*0.01<<" \t"<<thirdQuantile<<" \t"<<max<<" \t"<<round(std*100)*0.01<<endl;

    double rms = sqrt( sq_sum/N );
    //cout<<"N: \t"<<N<<endl;
//    cout<<"rms: \t"<<rms<<endl;
//    cout<<"avg: \t"<<mean<<endl;
//    //cout<<"median: \t"<<median<<endl;
//    cout<<"std: \t"<<std<<endl;
//    cout<<"min: \t"<<min<<endl;
//    cout<<"max: \t"<<max<<endl;
    Vector6d d; d<<rms,mean,median,std,min,max;
    return d;
}

#include <string>
std::string summary2(Vector6d rpe,int width){
    std::stringstream ss;
    ss<<std::setprecision(width);
    ss.setf(ios::fixed,ios::floatfield);
    //ss<<std::setw(width+2);
    ss<<" & "<<rpe(0)<<" & "<<rpe(1)<<" & "<<rpe(2)<<" & "<<rpe(3)<<" & "<<rpe(4)<<" & "<<rpe(5);
    return ss.str();
}

std::string printErrors(std::pair<Vector6d,Vector6d> rpeAte,double scale=1000,int width=3){
//             cout<<"RPE("<<rpes.size()<<")"<<"rms,mean,median,std,min,max"<<endl;
//             cout<<"ATE("<<rpes.size()<<")"<<"rms,mean,median,std,min,max"<<endl;
             //cout<<std::fixed;
             //cout<<std::setprecision(width);//<<std::setw(5);
             std::stringstream ss;
             ss<<summary2(rpeAte.first*scale,width)<<endl;
             ss<<summary2(rpeAte.second*scale,width)<<"\\\\\ \\\cline{2-14}"<<endl;
             return ss.str();
             //cout<<rpe(0)<<" & "<<rpe(1)<<" & "<<rpe(2)<<" & "<<rpe(3)<<" & "<<ate(0)<<" & "<<ate(1)<<" & "<<ate(2)<<" & "<<ate(3)<<"\\\ /\cline{2-10}"<<endl;
}

std::string printImpr(std::pair<Vector6d,Vector6d> A, std::pair<Vector6d,Vector6d> B){
//             cout<<"RPE("<<rpes.size()<<")"<<"rms,mean,median,std,min,max"<<endl;
//             cout<<"ATE("<<rpes.size()<<")"<<"rms,mean,median,std,min,max"<<endl;
             //cout<<std::setprecision(5)<<std::setw(5);
             Vector6d RPE_impr = (A.first-B.first).array()/A.first.array();
             Vector6d ATE_impr = (A.second-B.second).array()/A.second.array();
//             RPE_impr *=100;
//             ATE_impr *=100;
//             ATE_impr =

             std::pair<Vector6d,Vector6d> rpeAteImpr = std::make_pair(RPE_impr,ATE_impr);
             return printErrors(rpeAteImpr,100,0);
//             cout<<"RPE_impr"<<RPE_impr.transpose()<<endl;
//             cout<<"ATE_impr"<<ATE_impr.transpose()<<endl;
             //cout<<rpe(0)<<" & "<<rpe(1)<<" & "<<rpe(2)<<" & "<<rpe(3)<<" & "<<ate(0)<<" & "<<ate(1)<<" & "<<ate(2)<<" & "<<ate(3)<<"\\\ /\cline{2-10}"<<endl;
}

void saveImage(string pre, char key){


    int width=Visualize::WINDOW_WIDTH;
    int height=Visualize::WINDOW_HEIGHT;

    unsigned char* buffer = new unsigned char[width*height*3];
    glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, buffer);
    cv::Mat image(height, width, CV_8UC3, buffer);
    cv::flip(image, image, 0);
    //image=image.t();

    cv::namedWindow("screenshot");
    cv::imshow("screenshot", image);

    std::stringstream ss;
    ss<<Params::getDir()<<"/results_"<<pre<<"_"<<key<<".png";
    imwrite(ss.str(), image );
    cout<<"saved "<<ss.str()<<endl;
    cv::waitKey(1);
}

int main(int argc, char * argv[])
{

    bool directICP = false;
    bool showDepthMap = false;
    int step = 1;
    int nFrames = 1; //number of last frames for ppf matching
    int knn=2; //knn poses
    int iter=100;

    int limit = 1000;

    bool useLevenberg = false; //dogleg otherwise


    getParam("knn",knn,argc,argv);
    getParam("iter",iter,argc,argv);
    getParam("directICP", directICP, argc, argv);
    getParam("showDepthMap", showDepthMap, argc, argv);
    getParam("step", step, argc, argv);
    getParam("nFrames", nFrames, argc, argv);
    getParam("limit",limit,argc,argv);
    getParam("useLevenberg",useLevenberg,argc,argv);


    Params* inst = Params::getInstance();
    inst->getParams(argc,argv);

    float cutoff=inst->ddist;//0.01f;
    getParam("cutoff", cutoff, argc, argv);

    float cutoffGlobal=inst->ddist;//0.01f;
    getParam("cutoffGlobal", cutoffGlobal, argc, argv);


    float huberWidth=-1;//inst->ddist*0.5f;
    getParam("huberWidth", huberWidth, argc, argv);


    vector< std::shared_ptr<PointCloud> > frames;

    std::function<std::pair<Vector6d,Vector6d> ()> e = [&]() -> std::pair<Vector6d,Vector6d>{

            int delta = 1;
            vector<double> rpes = PointCloudManipulation::rpeVector(frames,delta);
            //Map<VectorXd> rpesV(&rpes[0],rpes.size()); //maps vector<Vector3f>
            //cout<<"rpe "<<rpesV.transpose()<<endl;
            Vector6d rpe=summary(rpes);


            Isometry3d S = PointCloudManipulation::leastSquaresEstimatedTrajectoryOntoGroundTruth(frames);
            Visualize::setS(S.cast<float>());
            vector<double> ates = PointCloudManipulation::ateVector(frames,S);
            Vector6d ate=summary(ates);

            return std::make_pair(rpe,ate);
    };

    std::function<void ()> f = [&]() -> void{
             std::pair<Vector6d,Vector6d> rpeAte = e();
             printErrors(rpeAte);

             //rpe.transpose()<<" & "<<ate.transpose()<<endl;
            //LoadingSaving::savePosesEvalutation(frames);  //saves P and Q
     };

    Visualize::setCallbackForKey('k',f);

    string S="A";

    for (char c : {'x','X','y','Y','z','Z'}) {
        Visualize::setCallbackForKey(c,[&,c]() -> void{
              saveImage(S,c);
        });
    }

    Visualize::setCallbackForKey('a',[&]() -> void{
        Visualize::getInstance()->setOffset((-1)*(frames[0]->pose*frames[0]->centerOfMass));
    });


    ApproachComponents::preprocessing(frames,step,showDepthMap,limit);

    std::pair<Vector6d,Vector6d> A,B,C;

    if(directICP){
        S="B";
        cout<<"press for pairwise alignment and refinment"<<endl;
        Visualize::spin();
        ApproachComponents::pairwiseAlignmentAndRefinement(frames,nFrames,cutoff);
        B = e();
        cout<<"& B"<<endl<<printErrors(B);
    }else{
        cout<<"press for pairwise coarse alignment"<<endl;
        Visualize::spin();
        ApproachComponents::pairwiseAlignment(frames,nFrames);
        A = e();
        cout<<"& A"<<endl<<printErrors(A);

        cout<<"press for pairwise refinement"<<endl;
        Visualize::spin();
        S = "B";
        ApproachComponents::pairwiseRefinement(frames,cutoff);
        B = e();
        cout<<"& A"<<endl<<printErrors(A);
        cout<<"& $\\downarrow$"<<endl<<printImpr(A,B);
        cout<<"& B"<<endl<<printErrors(B);
    }

    cout<<"press for global refinement"<<endl;
    Visualize::spin();
    S ="C";
    ApproachComponents::g2oOptimizer(frames,cutoffGlobal,iter,knn,huberWidth,useLevenberg);
    C = e();

    {
        stringstream ss;
        if(!directICP){
            ss<<"& A"<<endl<<printErrors(A);
            ss<<"& $\\downarrow$"<<endl<<printImpr(A,B);
        }
        ss<<"& B"<<endl<<printErrors(B);
        ss<<"& $\\downarrow$"<<endl<<printImpr(B,C);
        ss<<"& C"<<endl<<printErrors(C);


        std::stringstream filename;
        filename<<Params::getDir()<<"/results.txt";

        std::ofstream outputFile(filename.str()) ;
        outputFile<<ss.str();
        outputFile.close();

        cout<<ss.str()<<endl;
        cout<<"saved results to: "<<filename.str()<<endl;
    }


    {
        std::stringstream ss;
        for(int i=0; i<argc; i++)
        {
            if (argv[i][0]!='-') continue;
            if (argv[i][1]=='d' && argv[i][2]=='i' && argv[i][3]=='r') continue;
            ss << argv[i] <<" "<< argv[i+1]<<"\\\\"<<endl;
        }
        //cout<<"params: "<<endl<<ss.str()<<endl;
        std::stringstream filename;
        filename<<Params::getDir()<<"/results_params.txt";

        std::ofstream outputFile(filename.str()) ;
        outputFile<<ss.str();
        outputFile.close();

        cout<<ss.str()<<endl;
        cout<<"saved params to: "<<filename.str()<<endl;

    }
    Visualize::spinLast();

}
