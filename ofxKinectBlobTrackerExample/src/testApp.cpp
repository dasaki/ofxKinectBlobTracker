#include "testApp.h"

#define COUNTDOWN 3000
//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

    // enable depth->rgb image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();

	angle = kinect.getCurrentCameraTiltAngle();


    blobFinder.init(&kinect, true); // standarized coordinate system: z in the direction of gravity
    blobFinder.setResolution(BF_LOW_RES);
    blobFinder.setRotation( ofVec3f( angle, 0, 0) );
    blobFinder.setTranslation(ofVec3f(0,0,0));
    blobFinder.setScale(ofVec3f(0.001, 0.001, 0.001)); // mm to meters
    // bind our kinect to the blob finder
    // in order to do this we need to declare in testApp.h: class testApp : public ofBaseApp, public ofxKinectBlobListener
    blobTracker.setListener( this );

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	bgImage.allocate(kinect.width, kinect.height);
    grayDiff.allocate(kinect.width, kinect.height);
    grayDiffOfImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

    numPixels = kinect.width*kinect.height;

	// NOTE: measurement units in meters!!!
	minBlobVol = 0.02f;
	maxBlobVol = 2.0f;
    //no cropping
    cropBoxMin = ofVec3f(-10, -10, -10);
    cropBoxMax = ofVec3f(10, 10, 10);
    //
    thresh3D = ofVec3f(0.2,0.2,0.3);
    // xy pixel search range
    thresh2D = 1;
    maxBlobs = 10;

    float sqrResolution = blobFinder.getResolution();
    sqrResolution *= sqrResolution;
    minBlobPoints = (int)(0.001*(float)numPixels/sqrResolution);

    printf("min %f\n", minBlobVol);

	bLearnBakground = true;
	timeLapse = 0;
	font.loadFont("PerfectDOSVGA437.ttf",15, true, true, true);

	ofSetFrameRate(60);

	// start from the front
	bDrawPointCloud = false;
	bDrawIDs = true;
}

//--------------------------------------------------------------
void testApp::update() {

	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        // background subtraction
        if ((bLearnBakground) && (ofGetElapsedTimeMillis() >= timeLapse)) {
            bgImage = grayImage;   // let this frame be the background image from now on
            bLearnBakground = false;
        }
        cvAbsDiff(bgImage.getCvImage(), grayImage.getCvImage(), grayDiff.getCvImage());
        cvErode(grayDiff.getCvImage(), grayDiff.getCvImage(), NULL, 2);
        cvDilate(grayDiff.getCvImage(), grayDiff.getCvImage(), NULL, 1);
        // threshold ignoring little differences
        cvThreshold(grayDiff.getCvImage(), grayDiff.getCvImage(), 3, 255, CV_THRESH_BINARY);
        grayDiff.flagImageChanged();
        // update the ofImage to be used as background mask for the blob finder
        grayDiffOfImage.setFromPixels(grayDiff.getPixels(), kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

        blobFinder.findBlobs( &grayDiffOfImage,
                           cropBoxMin, cropBoxMax,
                           thresh3D, thresh2D,
                           minBlobVol, maxBlobVol, minBlobPoints,  maxBlobs);
        blobTracker.trackBlobs( blobFinder.blobs );
	}

#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {

	ofSetColor(255, 255, 255);

	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, 320, 240);
		kinect.draw(340, 10, 320, 240);

		grayImage.draw(10, 260, 320, 240);

		grayDiff.draw(340, 260, 320, 240);

	}

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "press b to set current frame as background" << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "num blobs found " << blobFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press i to toggle draw blob IDs in 3D scene" << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(),20,510);

    char timeLapseStr[256];
    if (bLearnBakground) {
        ofSetHexColor(0xffffff);
        sprintf( timeLapseStr, "%i", timeLapse-ofGetElapsedTimeMillis() );
        ofPushMatrix();
        font.drawString(timeLapseStr, ofGetWidth()/2, (ofGetHeight()-font.getLineHeight())/2);
        ofPopMatrix();
    }
}
//--------------------------------------------------------------
void testApp::drawPointCloud() {
    ofPushMatrix();
    ofScale(100.0,100.0,100.0);
    glEnable(GL_DEPTH_TEST);
    glPointSize(3);
    // draw blobs
    for (unsigned int i=0; i < blobFinder.blobs.size(); i++) {
        ofSetColor(25*i,25*i,255-25*i);
        // draw blobs
        blobFinder.blobs[i].draw();
        // plot blobs IDs
        if (bDrawIDs) {
            ofPushMatrix();
            ofTranslate(blobTracker.blobs[i].massCenter.x, blobTracker.blobs[i].massCenter.y, blobTracker.blobs[i].maxZ.z);
            ofRotateX(-90);
            ofScale(0.01f, 0.01f, 0.01f);
            ofSetColor(255,255,255);
            font.drawStringAsShapes(ofToString(blobTracker.blobs[i].id), 0, 0);
            ofPopMatrix();
        }
        // draw trajectory as a line
        vector <ofVec3f> trajectory;
        blobTracker.getTrajectoryById(blobTracker.blobs[i].id, trajectory);
        unsigned int trjSize = trajectory.size();
        if (trjSize > 1) {
            ofPushMatrix();
            ofSetColor(255,255,0);
            ofSetLineWidth(3);
            glBegin(GL_LINE);
            for (unsigned int j = 0; j < trjSize; j++) {
                glVertex3f( trajectory[j].x, trajectory[j].y, trajectory[j].z );
            }
            glEnd();
            ofPopMatrix();
            trajectory.clear();
        }
    }
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
}
//--------------------------------------------------------------
void testApp::exit() {
	kinect.close();

#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {

		case 'b':
            bLearnBakground = true;
            // to give you time to run away of the field of view
            timeLapse = COUNTDOWN+ofGetElapsedTimeMillis();
            break;

        case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;

        case'i':
			bDrawIDs = !bDrawIDs;
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;

		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			blobFinder.setRotation( ofVec3f( angle, 0, 0) );
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			blobFinder.setRotation( ofVec3f( angle, 0, 0) );
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
/*
 *
 *	blob section
 *
 *	from here on in it's blobs
 *	thanks to stefanix and the opencv library :)
 *
 */

//--------------------------------------------------
void testApp::blobOn( ofVec3f centroid, int id, int order ) {
   // cout << "blobOn() - id:" << id << " order:" << order << endl;
}

void testApp::blobMoved( ofVec3f centroid, int id, int order) {
  //  cout << "blobMoved() - id:" << id << " order:" << order << endl;
  // full access to blob object ( get a reference)
  //  ofxKinectTrackedBlob blob = blobTracker.getById( id );
  // cout << "volume: " << blob.volume << endl;
}

void testApp::blobOff( ofVec3f centroid, int id, int order ) {
   // cout << "blobOff() - id:" << id << " order:" << order << endl;
}
