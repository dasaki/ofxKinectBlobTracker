/*
* ofxCvBlobTracker.h
* 
* Purpose: 3D blob tracker to be used with ofxKinectBlobFinder and the kinect 3D sensor
*
* Author: David Sanz Kirbis, May 2012
*
* Comments:
*
* Part of the cvCinema project: www.cvcinema.com
*
* Based on ofxCvTracker by stefanix
*
******************************************************************************/


#ifndef OF_KINECT_BLOBTRACKER_H
#define OF_KINECT_BLOBTRACKER_H

#include <map>
#include <vector>
#include "ofMain.h"

#include "ofxKinectTrackedBlob.h"
#include "ofxKinectTrackingConstants.h"



class ofxKinectBlobTracker {


  public:

	vector<ofxKinectTrackedBlob>  blobs;


    ofxKinectBlobTracker();
    void setListener( ofxKinectBlobListener* _listener );
    void trackBlobs( const vector<ofxKinectBlob>& blobs);
    int findOrder( int id );  // order by which the present
                              // blobs came into existence
    int getIndexById( int id );  // returns a reference to the
                                         // corresponding blob in blobs vector
    void getTrajectoryById( int id, vector <ofVec3f> &trajectory);
    void draw( );  // draws all blobs
	ofVec3f findVelocity(unsigned int index );
    ofVec3f findVelocityById( int id );


    float getRejectDistThresh();
    unsigned int getNumGhostFrames();
    float getMinDisplacementThresh();

    void setRejectDistThresh(float newValue);
    void setNumGhostFrames(unsigned int  newValue);
    void setMinDisplacementThresh(float newValue);
    
  protected:

    int currentID;
    int extraIDs;
    unsigned int numcheck;

    ofxKinectBlobListener* listener;

    float rejectDistThresh;
    int numGhostFrames;
    float minDisplacementThresh;

    vector<vector<int> > matrix;
    vector<int> ids;
    vector<vector<ofxKinectTrackedBlob> > history;


    void doBlobOn( const ofxKinectTrackedBlob& b );
    void doBlobMoved( const ofxKinectTrackedBlob& b );
    void doBlobOff( const ofxKinectTrackedBlob& b );

    inline void permute( unsigned int k );
    inline bool checkValid( int start );
    inline bool checkValidNew( int start );
};



#endif

