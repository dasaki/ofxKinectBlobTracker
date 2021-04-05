#include "ofApp.h"

#define COUNTDOWN 3000
#define AXIS_LEN        0.5

#define KIN_3D_W        0.05
#define KIN_3D_H        0.05
#define KIN_3D_L        0.3
#define CAM_3D_SPH_RAD  0.035
#define CAM_3D_BASE     0.1
#define CAM_3D_CONE_H   0.15
#define CAM_3D_CONE_BASE 0.1
#define MIN_DEPTH       0.3
#define MAX_DEPTH       8.0
#define K_VFOV          43
#define K_VFOV_2_RAD    0.375245789f
#define K_HFOV          57
#define K_HFOV_2_RAD    0.497418837f
#define FHH             MAX_DEPTH/cosf(K_HFOV_2_RAD)
#define FVH             MAX_DEPTH/cosf(K_VFOV_2_RAD)
#define K_FAR           ofVec3f(FHH*sinf(K_HFOV_2_RAD),MAX_DEPTH,FVH*sinf(K_VFOV_2_RAD))

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

    // enable depth->rgb image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();

	tiltAngle = kinect.getCurrentCameraTiltAngle();


    blobFinder.init(&kinect, true); // standarized coordinate system: z in the direction of gravity
    blobFinder.setResolution(BF_LOW_RES);
    blobFinder.setRotation( ofVec3f( tiltAngle, 0, 0) );
    blobFinder.setTranslation(ofVec3f(0,0,0));
    blobFinder.setScale(ofVec3f(0.001, 0.001, 0.001)); // mm to meters
    // bind our kinect to the blob finder
    // in order to do this we need to declare in ofApp.h: class ofApp : public ofBaseApp, public ofxKinectBlobListener
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
//	minBlobVol = 0.02f;
//	maxBlobVol = 2.0f;
    //no cropping
   // cropBoxMin = ofVec3f(-10, -10, -10);
   // cropBoxMax = ofVec3f(10, 10, 10);
    //
  //  thresh3D = ofVec3f(0.1,0.1,0.2);
    // xy pixel search range
  //  thresh2D = 1;
  //  maxBlobs = 10;

    minBlobPoints = (int)(0.001*(float)numPixels/blobFinder.getResolution());

  
	bLearnBakground = true;
	timeLapse = COUNTDOWN+ofGetElapsedTimeMillis();
	font.load("PerfectDOSVGA437.ttf",15, true, true, true);

	ofSetFrameRate(60);

	bDrawIDs = true;
	
	
	gui.setup(); // most of the time you don't need a name
	
	gui.add( numBlobsLabel.set( "Blobs Found", "") );
	gui.add(tiltAngle.set( "Tilt angle", kinect.getCurrentCameraTiltAngle(), -30, 30 ));
    gui.add(minDisplacementThresh.set("minDisplacementThresh", blobTracker.getRejectDistThresh(), 0.001f, 10.0f));
	gui.add(rejectDistThresh.set("rejectDistThresh", blobTracker.getNumGhostFrames(), 0.001f, 10.0f));
	gui.add(numGhostFrames.set("numGhostFrames", blobTracker.getMinDisplacementThresh(), 0, 30));
	gui.add(maxBlobs.set("maxBlobs", 10 , 1, 100));    
	gui.add(minBlobVol.set("minBlobVol", 0.001f, 0.0001f, 9.9f));
	gui.add(maxBlobVol.set("maxBlobVol", 2.0f, 0.002f, 10.0f));
	
	
	gui.add(thresh2D.set("thresh2D", 1 , 1, 10));
	gui.add(thresh3D.set("thresh3D", glm::vec3(0.1,0.1,0.2) , glm::vec3(0.01,0.01,0.01), glm::vec3(10,10,10)));
	gui.add(cropBoxMin.set("cropBoxMin", glm::vec3(-3,MIN_DEPTH,-2) , glm::vec3(-5,0,-5), glm::vec3(5,MAX_DEPTH,5)));
	gui.add(cropBoxMax.set("cropBoxMax", glm::vec3(3,8,2) , glm::vec3(-5,0,-5), glm::vec3(5,MAX_DEPTH,5)));
	updateCropBox();
	gui.add(viewMode.setup("Cam/Pointcloud", false));
	   
	cropBoxMin.addListener(this, &ofApp::cropBoxChanged);
    cropBoxMax.addListener(this, &ofApp::cropBoxChanged);
	
    minBlobVol.addListener(this, &ofApp::minBlobVolChanged);
    maxBlobVol.addListener(this, &ofApp::maxBlobVolChanged);
    tiltAngle.addListener(this, &ofApp::tiltAngleChanged);
    minDisplacementThresh.addListener(this, &ofApp::minDisplacementThreshChanged);
	rejectDistThresh.addListener(this, &ofApp::rejectDistThreshChanged);
	numGhostFrames.addListener(this, &ofApp::numGhostFramesChanged);
	
	 /*   ofParameter<long unsigned int> timeLapse;
    ofParameter<float> minBlobVol;
    ofParameter<float> maxBlobVol;
    ofParameter<glm::vec3> cropBoxMin;
    */
    
	
	bHide = false;
	
	
	
}

//--------------------------------------------------------------
void ofApp::update() {

	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels().getData(), kinect.width, kinect.height);
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
        grayDiffOfImage.setFromPixels(grayDiff.getPixels().getData(), kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

        blobFinder.findBlobs( &grayDiffOfImage,
                           ofVec3f(cropBoxMin->x,cropBoxMin->y,cropBoxMin->z) ,
                           ofVec3f(cropBoxMax->x,cropBoxMax->y,cropBoxMax->z) ,
                           ofVec3f(thresh3D->x,thresh3D->y,thresh3D->z) ,
                           thresh2D,
                           minBlobVol, maxBlobVol, minBlobPoints,  maxBlobs);
        numBlobsLabel.set( ofToString(blobFinder.nBlobs));
        blobTracker.trackBlobs( blobFinder.blobs );
	}

#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {

	ofSetColor(255, 255, 255);

	if(viewMode) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
	    ofPushMatrix();
	    ofTranslate(gui.getWidth()+10,0,0);
		// draw from the live kinect
		kinect.drawDepth(10, 10, 320, 240);
		kinect.draw(340, 10, 320, 240);

		grayImage.draw(10, 260, 320, 240);

		grayDiff.draw(340, 260, 320, 240);
        ofPopMatrix();
	}


    if(!bHide){
		    gui.draw();
	    
	    // draw instructions
	    /*ofSetColor(255, 255, 255);
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
	    ofDrawBitmapString(reportStream.str(),20,510);*/
    }
    char timeLapseStr[256];
    if (bLearnBakground) {
        ofSetHexColor(0xffffff);
        sprintf( timeLapseStr, "%lu", timeLapse-ofGetElapsedTimeMillis() );
        ofPushMatrix();
        font.drawString(timeLapseStr, ofGetWidth()/2, (ofGetHeight()-font.getLineHeight())/2);
        ofPopMatrix();
    }
}
//--------------------------------------------------------------
void ofApp::drawPointCloud() {
    ofPushMatrix();
    ofRotateXDeg(-90);
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
            ofRotateXDeg(90);
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
            ofSetLineWidth(1);
            ofPopMatrix();
            trajectory.clear();
        }
    }
      

  draw3Dgui();
    glDisable(GL_DEPTH_TEST);
    ofPopMatrix();
}
/************************************************************
                    DRAW 3D GUI
*************************************************************/
void ofApp::draw3Dgui() {

    ofPushMatrix();
    glPushMatrix();
        glPointSize(1);
    // draw room box
    glColor3ub((unsigned char)255,(unsigned char)128,(unsigned char)128);
    drawGlWireBox(ofVec3f(cropBoxMin->x, cropBoxMin->y, cropBoxMin->z), ofVec3f(cropBoxMax->x, cropBoxMax->y, cropBoxMax->z), false); 
	// draw coordinate system
    draw3Daxis(ofPoint(0,0,0),ofPoint(0,0,0), AXIS_LEN);
    // draw kinect potition
    ofPushMatrix();


  
    // kinect axis
    draw3Daxis(ofPoint(0,0,0),ofPoint(0,0,0), AXIS_LEN);
    // kinect camera ranges and FOV
    glBegin(GL_LINES);
    glColor3ub((unsigned char)128,(unsigned char)128,(unsigned char)255);
    glVertex3f(0, 0, 0);
    glVertex3f(K_FAR.x, K_FAR.y, K_FAR.z);
    glVertex3f(0, 0, 0);
    glVertex3f(-K_FAR.x, K_FAR.y, K_FAR.z);
    glVertex3f(0, 0, 0);
    glVertex3f(K_FAR.x, K_FAR.y, -K_FAR.z);
    glVertex3f(0, 0, 0);
    glVertex3f(-K_FAR.x, K_FAR.y, -K_FAR.z);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(K_FAR.x, K_FAR.y, K_FAR.z);
    glVertex3f(K_FAR.x, K_FAR.y, -K_FAR.z);
    glVertex3f(-K_FAR.x, K_FAR.y, -K_FAR.z);
    glVertex3f(-K_FAR.x, K_FAR.y, K_FAR.z);
    glEnd();
    
    ofPopMatrix();
   


    glPopMatrix();
    ofPopMatrix();
}
/************************************************************
                    DRAW 3D AXIS
*************************************************************/
void ofApp::draw3Daxis(ofPoint tran, ofPoint rot, float axisLen) {
    ofPushMatrix();
    ofTranslate(tran.x, tran.y, tran.z);

    ofRotateXDeg(rot.x);
    ofRotateYDeg(rot.y);
    ofRotateZDeg(rot.z);

    glBegin(GL_LINES);
    glColor3ub((unsigned char)255,(unsigned char)0,(unsigned char)0);
	glVertex3f(0, 0, 0);
    glVertex3f(axisLen, 0, 0);
    glColor3ub((unsigned char)0,(unsigned char)255,(unsigned char)0);
	glVertex3f(0, 0, 0);
    glVertex3f(0, axisLen, 0);
    glColor3ub((unsigned char)0,(unsigned char)0,(unsigned char)255);
	glVertex3f(0, 0, 0);
    glVertex3f(0, 0, axisLen);
    glEnd();
    ofPopMatrix();
}

/************************************************************
                    DRAW GL WIRE BOX
*************************************************************/
void ofApp::drawGlWireBox(ofVec3f minPoint, ofVec3f maxPoint, bool centerCross)
{
    // draw bounding rectangular parallelepiped
    glBegin(GL_LINE_LOOP);
    glVertex3f(minPoint.x, minPoint.y, minPoint.z);
    glVertex3f(minPoint.x, maxPoint.y, minPoint.z);
    glVertex3f(minPoint.x, maxPoint.y, maxPoint.z);
    glVertex3f(minPoint.x, minPoint.y, maxPoint.z);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(maxPoint.x, minPoint.y, minPoint.z);
    glVertex3f(maxPoint.x, maxPoint.y, minPoint.z);
    glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
    glVertex3f(maxPoint.x, minPoint.y, maxPoint.z);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(minPoint.x, minPoint.y, minPoint.z);
    glVertex3f(maxPoint.x, minPoint.y, minPoint.z);
    glVertex3f(minPoint.x, maxPoint.y, minPoint.z);
    glVertex3f(maxPoint.x, maxPoint.y, minPoint.z);
    glVertex3f(minPoint.x, maxPoint.y, maxPoint.z);
    glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
    glVertex3f(minPoint.x, minPoint.y, maxPoint.z);
    glVertex3f(maxPoint.x, minPoint.y, maxPoint.z);
    if (centerCross) {
        //draw crossing lines marking centroid
        glVertex3f(minPoint.x, minPoint.y, minPoint.z);
        glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
        glVertex3f(maxPoint.x, minPoint.y, minPoint.z);
        glVertex3f(minPoint.x, maxPoint.y, maxPoint.z);
        glVertex3f(minPoint.x, minPoint.y, maxPoint.z);
        glVertex3f(maxPoint.x, maxPoint.y, minPoint.z);
        glVertex3f(maxPoint.x, minPoint.y, maxPoint.z);
        glVertex3f(minPoint.x, maxPoint.y, minPoint.z);
    }
    glEnd();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.close();

#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
        case 'h':
		        bHide = !bHide;
                break;
        case's':
		        gui.saveToFile("settings.xml");
                break;
        case'l':
		    gui.loadFromFile("settings.xml");
		    break;
		
		case 'b':
            bLearnBakground = true;
            // to give you time to run away of the field of view
            timeLapse = COUNTDOWN+ofGetElapsedTimeMillis();
            break;

        case'v':
			viewMode = !viewMode;
			break;

        case'i':
			bDrawIDs = !bDrawIDs;
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;

		case 'o':
			kinect.setCameraTiltAngle(tiltAngle); // go back to prev tilt
			blobFinder.setRotation( ofVec3f( tiltAngle, 0, 0) );
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			blobFinder.setRotation( ofVec3f( tiltAngle, 0, 0) );
			kinect.close();
			break;

		case 'q':
			tiltAngle++;
			if(tiltAngle>30) tiltAngle=30;
			kinect.setCameraTiltAngle(tiltAngle);
			blobFinder.setRotation( ofVec3f( tiltAngle, 0, 0) );
			break;

		case 'a':
			tiltAngle--;
			if(tiltAngle<-30) tiltAngle=-30;
			kinect.setCameraTiltAngle(tiltAngle);
			blobFinder.setRotation( ofVec3f( tiltAngle, 0, 0) );
			break;
	}
}

//--------------------------------------------------------------
void ofApp::cropBoxChanged(glm::vec3 &parameter){
    updateCropBox();
}

//--------------------------------------------------------------
void ofApp::updateCropBox() {
    box.set( cropBoxMax->x - cropBoxMin->x,  cropBoxMax->y - cropBoxMin->y,  cropBoxMax->z - cropBoxMin->z );
	box.setPosition( cropBoxMin->x+box.getWidth()/2,  cropBoxMin->y+box.getHeight()/2,  cropBoxMin->z+box.getDepth()/2);
}
//--------------------------------------------------------------
void ofApp::minBlobVolChanged(float &minBlobVol){
	if (minBlobVol >= maxBlobVol && maxBlobVol < 9.9) maxBlobVol = minBlobVol + 0.001; 
}
//--------------------------------------------------------------
void ofApp::maxBlobVolChanged(float &maxBlobVol){
	if (maxBlobVol <= minBlobVol && minBlobVol > 0.002) minBlobVol = maxBlobVol - 0.001; 
}
//--------------------------------------------------------------
void ofApp::tiltAngleChanged(int &tiltAngle){
	kinect.setCameraTiltAngle(tiltAngle);
	blobFinder.setRotation( ofVec3f( tiltAngle, 0, 0) );
}
//--------------------------------------------------------------
void ofApp::minDisplacementThreshChanged(float &minDisplacementThresh){
	blobTracker.setMinDisplacementThresh(minDisplacementThresh);
}
//--------------------------------------------------------------
void ofApp::rejectDistThreshChanged(float &rejectDistThresh){
	blobTracker.setRejectDistThresh(rejectDistThresh);
}
//--------------------------------------------------------------
void ofApp::numGhostFramesChanged(unsigned int &numGhostFrames){
	blobTracker.setRejectDistThresh(numGhostFrames);
}
//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
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
void ofApp::blobOn( ofVec3f centroid, int id, int order ) {
   // cout << "blobOn() - id:" << id << " order:" << order << endl;
}

void ofApp::blobMoved( ofVec3f centroid, int id, int order) {
  //  cout << "blobMoved() - id:" << id << " order:" << order << endl;
  // full access to blob object ( get a reference)
  //  ofxKinectTrackedBlob blob = blobTracker.getById( id );
  // cout << "volume: " << blob.volume << endl;
}

void ofApp::blobOff( ofVec3f centroid, int id, int order ) {
   // cout << "blobOff() - id:" << id << " order:" << order << endl;
}
