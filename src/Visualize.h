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
  #include <OpenGL/glu.h>
  #include <GL/freeglut.h>
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
    static void update();
    static bool waitKey(unsigned char key); //wait in another thread for keypress in opengl window
    static void waitKeyQuit(); //waits till q is pressed, joins the threads

    static void visualize();  //blocks

    //mimics opencv's Viz
    static void spin();
    static void spin(int iterations);
    //static void spinOnce(int millis);

    //TODO make private, make better interface
    static Visualize* getInstance();

    KeyBucketPairList b;
    PointCloud model,scene,modelT;
    Matches matches;
    vector< pair<PointCloud, RowVector3f> > ms;
    vector<Isometry3f> cameraPoses;

    vector<int> closestPtsSceneToModel;

    vector<Vector3f> src,dst;

    int current_object;

    void drawFrustumIntrinsics();



private:
    Visualize(); // singleton, acces via factory

    bool keyToggle[256];

    static bool isInitalized;


    int modifier;
    
    GLfloat angle;   /* in degrees */
    GLfloat angle2;   /* in degrees */
    GLfloat angle3;   /* in degrees */
    GLfloat zoom;
    int mouseButton;
    int moving, startx, starty;
    
    GLfloat offsetY,offsetX;
    
    const static int WINDOW_SIZE = 800;
    
    void bucketInfo();

    static Visualize *instance;
    
    //MatrixXf m;

    unsigned char lastKey;

    std::thread* visThread;

    bool waitKeyInst(unsigned char key);

    int bucketIndex;
    int matchIndex;

    void glColorHex(int rgbHex);


    void drawCylinderAdvanced(double r, double l, bool coverback, bool coverfront, bool normalInwards);
    void drawCylinder(double r, double l);
    void drawOrigin();

    void drawNormals(PointCloud m, RowVector3f color);
    void drawPointCloud(PointCloud C, RowVector3f color, float pointSize = 4.0f);

    void drawPoints(vector<Vector3f> pts, vector<RowVector3f> color, float pointSize = 4.0f);
    void drawLines(vector<int> vertices);
    void drawLines(vector<Vector3f> v1, vector<Vector3f> v2);
    void drawCubes(vector<Vector3f> C, double size);
    void drawSpheres(vector<Vector3f> C, double radius);

    void drawPPF(int i, int j, PointCloud m);
    void drawPPfs(Bucket, PointCloud m);



    void drawMatches(Matches matches);
    //void printMatches(Matches matches);


    void drawAll(PointCloud m, RowVector3f color, RowVector3f colorNormals);



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

    static void idleW(void);

};

#endif /* defined(__PointPairFeatures__Visualize__) */

