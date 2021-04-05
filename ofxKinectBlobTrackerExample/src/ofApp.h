#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKinectBlobFinder.h"
#include "ofxKinectBlobTracker.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp, public ofxKinectBlobListener {
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
    void tiltAngleChanged(int &tiltAngle);
    void minDisplacementThreshChanged(float &minDisplacementThresh);
    void rejectDistThreshChanged(float &rejectDistThresh);
    void numGhostFramesChanged(unsigned int &numGhostFrames);
    void minBlobVolChanged(float &minBlobVol);
    void maxBlobVolChanged(float &maxBlobVol);
    void cropBoxChanged(glm::vec3 &parameter);
    void updateCropBox();
    void drawGlWireBox(ofVec3f minPoint, ofVec3f maxPoint, bool centerCross);
    void draw3Dgui();
    void draw3Daxis(ofPoint tran, ofPoint rot, float axisLen);

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

	bool bDrawIDs;
	bool bLearnBakground;
	bool bHide;

	long unsigned int timeLapse;
	ofTrueTypeFont font;

	//int angle;
	int numPixels;

	unsigned int minBlobPoints;

    
    ofBoxPrimitive box;
    
    ofParameter<float> minDisplacementThresh;
    ofParameter<float> rejectDistThresh;
    ofParameter<unsigned int> numGhostFrames;
    ofParameter<unsigned int> maxBlobs;
    ofParameter<int> thresh2D;
    ofParameter<int> tiltAngle;
    ofParameter<float> minBlobVol;
    ofParameter<float> maxBlobVol;
    ofParameter<glm::vec3> cropBoxMin;
    ofParameter<glm::vec3> cropBoxMax;
    ofParameter<glm::vec3>  thresh3D;
    ofParameter<string> numBlobsLabel;
    ofxToggle viewMode;
  
	/*ofxToggle filled;
	ofxButton */

	ofxPanel gui;





    // used for viewing the point cloud
	ofEasyCam easyCam;
};
