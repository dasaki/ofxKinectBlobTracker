#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKinectBlobFinder.h"
#include "ofxKinectBlobTracker.h"

class testApp : public ofBaseApp, public ofxKinectBlobListener {
public:

	void setup();
	void update();
	void draw();
	void exit();

	void drawPointCloud();

	void keyPressed (int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    void blobOn( ofVec3f centroid, int id, int order );
    void blobMoved( ofVec3f centroid, int id, int order);
    void blobOff( ofVec3f centroid, int id, int order );

	ofxKinect kinect;
    ofxKinectBlobFinder  blobFinder;
    ofxKinectBlobTracker blobTracker;

	ofxCvColorImage colorImg;

	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    ofxCvGrayscaleImage     bgImage;
    ofxCvGrayscaleImage 	grayDiff;
    // for KinectBlobTracker
    ofImage grayDiffOfImage;

	bool bDrawPointCloud;
	bool bDrawIDs;
	bool bLearnBakground;

	int timeLapse;
	ofTrueTypeFont font;

	int angle;
	int numPixels;

	int minBlobPoints;
	float minBlobVol;
	float maxBlobVol;

    ofVec3f cropBoxMin;
    ofVec3f cropBoxMax;
    ofVec3f thresh3D;
    int thresh2D;
    unsigned int maxBlobs;



    // used for viewing the point cloud
	ofEasyCam easyCam;
};
