//
//  Visualize.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 29.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef __PointPairFeatures__Visualize__
#define __PointPairFeatures__Visualize__

#include <iostream>
#include <math.h>
#include <vector>
#include <thread>

//WINDOWS
//#include <Windows.h>
//#define M_PI 3.141592654
//#include "GL\glut.h"

#if defined (__APPLE__) || defined(MACOSX)
 //MAC
 #include <OpenGL/gl.h>
 #include <GLUT/GLUT.h>
 #define glutLeaveMainLoop() //exit(0)
#else
 //LINUX
 #include "GL/freeglut.h"
 #include "GL/gl.h"
#endif

#include <eigen3/Eigen/Dense>
#include "Constants.h"

#define glRED glColor3f(0.0,1,0.2);


using namespace Eigen;
using namespace std;

class Visualize {

public:
    static void start();       //starts the visualisation async
    static void waitKey(unsigned char key); //wait in another thread for keypress in opengl window
    static void waitKeyQuit(); //waits till q is pressed, joins the threads

    //TODO: mimic opencv's Viz
    //static void spin();
    //static void spinOnce(int millis);

    //TODO make private, make better interface
    static Visualize* getInstance();

    KeyBucketPairList b;
    MatrixXd model,scene,modelT;
    Matches matches;
    vector< pair<MatrixXd, RowVector3f> > ms;


private:
    Visualize(); // singleton, acces via factory

    bool m_Smooth;
    bool m_Highlight;
    bool m_ColorMaterial;
    bool m_Emission;
    bool m_Ambient; //use white as ambient color, not the one set by colorMaterial
    bool m_Origin;
    bool m_Normals;
    bool m_Buckets;
    bool m_Voxels;

    int modifier;
    
    GLfloat angle;   /* in degrees */
    GLfloat angle2;   /* in degrees */
    GLfloat angle3;   /* in degrees */
    GLfloat zoom;
    int mouseButton;
    int moving, startx, starty;
    
    GLfloat offsetY,offsetX;
    
    const static int WINDOW_SIZE = 1024;
    int current_object;
    
    void bucketInfo();

    static Visualize *instance;
    
    MatrixXd m;

    unsigned char lastKey;

    std::thread* visThread;

    static void visualize();
    void waitKeyInst(unsigned char key);

    int bucketIndex;
    int matchIndex;

    void glColorHex(int rgbHex);


    void drawCylinderAdvanced(double r, double l, bool coverback, bool coverfront, bool normalInwards);
    void drawCylinder(double r, double l);
    void drawOrigin();

    void drawNormals(MatrixXd m,RowVector3f color);
    void drawPointCloud(MatrixXd m, RowVector3f color);
    void drawPPF(int i, int j, MatrixXd m);
    void drawPPfs(Bucket, MatrixXd m);
    void drawCubes(MatrixXd C, double size);
    void drawMatches(Matches matches);
    void printMatches(Matches matches);


    void drawAll(MatrixXd m, RowVector3f color, RowVector3f colorNormals);



    void drawEllipticParaboloid(double max, double a, double b, double c, bool backSideNormal);
    void drawHyperbolicParaboloid(double maxx, double maxy, double a, double b, double c, bool backSideNormal);

    double getRotationAngleApprox(double xdiff, double ydiff, double x, double y);


    void display(void);
    void keyboard (unsigned char key, int x, int y);
    void mouse(int button, int state, int x, int y);
    void motion(int x, int y);

    static void displayW(void);
    static void keyboardW (unsigned char key, int x, int y);
    static void mouseW(int button, int state, int x, int y);
    static void motionW(int x, int y);


    int mainVisualize(int argc, char **argv);
};

#endif /* defined(__PointPairFeatures__Visualize__) */

