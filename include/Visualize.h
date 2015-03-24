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
  //#include <OpenGL/glu.h>
  #include <GL/freeglut.h>
    //#include <GL/glxew.h>
#else
 //LINUX
 #include "GL/freeglut.h"
 #include "GL/gl.h"
#endif

#include <eigen3/Eigen/Dense>

#include "Params.h"

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
    static void spinToggle(int iterations);
    static void spinLast();

    static Visualize* getInstance();

    static void setClouds(vector< shared_ptr<PointCloud> >* mypair);

    static void setS(Isometry3f S); //least squares alignment of P to Q;

    static void setCallbackForKey(char key, std::function<void()> f);

    static void setSelectedIndex(int i);

    void setOffset(Vector3f offset);

    int selectedFrame=255;
    int selectedOutgoingEdgeIdx=255;
    int ingoingEdgeFrame=255;

    bool enableSaving=false;

    static void simulateKeypress(char key);

    //const static int WINDOW_SIZE = 800;
    const static int WINDOW_WIDTH = 768;//1024;
    const static int WINDOW_HEIGHT = 768;

private:
    int refPtIdx=0;
    Visualize(); // singleton, acces via factory

    vector< shared_ptr<PointCloud> >* ms;

    Isometry3f S; //aligns P to Q in least squares sense

    void drawFrustumIntrinsics(Vector4f colorLine, Vector4f colorPlane);

    bool keyToggle[256];//key toggle states
    std::function<void()> functions[256]; //functions to call on keys


    void readPose(char key);
    void writePose(char key);
    void readKeyToggle(char key);
    void writeKeyToggle(char key);

    static bool isInitalized;

    int modifier;
    
    GLfloat angle;   /* in degrees */
    GLfloat angle2;   /* in degrees */
    GLfloat angle3;   /* in degrees */
    GLfloat zoom;
    GLfloat offsetY;
    GLfloat offsetX;
    GLfloat offsetZ;


    int mouseButton;
    int moving, startx, starty;
    
    void bucketInfo();

    static Visualize *instance;
    
    unsigned char lastKey;

    std::thread* visThread;

    bool waitKeyInst(unsigned char key);

    int bucketIndex;
    int matchIndex;

    void glColorHex(int rgbHex);

    void drawCylinderAdvanced(double r, double l, bool coverback, bool coverfront, bool normalInwards);
    void drawCylinder(double r, double l);
    void drawOrigin();

    void drawNormals(const PointCloud* m, Vector3f& color);
    void drawPointCloud(PointCloud* C, int i);

    void drawPoints(const vector<Vector3f>& pts, const Vector3f& color, float pointSize = 4.0f);
    void drawPoints(const vector<Vector3f>& pts, const vector<Vector3f>& colors, float pointSize = 4.0f);

    void drawLines(const vector<Vector3f>& v1, const vector<Vector3f>& v2);
    void drawCubes(const vector<Vector3f>& C, double size);
    void drawSpheres(const vector<Vector3f>& C, double radius);

    void drawPPF(int i, int j,const PointCloud* m);
    //void drawLines(const vector<int>& vertices);

    void drawCameraPose(Isometry3f& P,int i,Vector4f& colorLine,Vector4f& colorPlane);
    void drawCameraPoses(vector<Isometry3f>& cameraPoses,Vector4f& colorLine, Vector4f& colorPlane);

    void drawEdges(int i);

    void drawAll(PointCloud* m, Vector3f color, Vector3f colorNormals);

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

