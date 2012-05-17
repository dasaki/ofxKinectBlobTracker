/*
* ofxKinectTrackedBlob.h based in ofxCvTrackedBlob.h
* openFrameworks
*
* This class represents a blob with inter-frame information.
* This includes a persistent id to assume a persistent identity over
* time.
*
*/


#ifndef OF_KINECT_TRACKEDBLOB_H
#define OF_KINECT_TRACKEDBLOB_H

#include "ofxKinectBlobFinder.h"


class ofxKinectTrackedBlob : public ofxKinectBlob {
  public:

    int id;

    ofVec3f deltaLoc;
    ofVec3f deltaLocTotal;
    ofVec3f predictedPos;
    float deltaVolume;
	ofVec3f velocity;

    // Used only by BlobTracker
    //
    bool markedForDeletion;
    int framesLeft;
    vector<float> error;
    vector<int> closest;  // ids of the closest points, sorted



    ofxKinectTrackedBlob() {
        id = -1;
        volume = 0.0f;
        dimensions = ofVec3f(0.0f,0.0f,0.0f);
        massCenter = ofVec3f(0.0f,0.0f,0.0f);
        deltaVolume = 0.0f;
        markedForDeletion = false;
        framesLeft = 0;
    }

    ofxKinectTrackedBlob( const ofxKinectBlob& b ) {
        volume = b.volume;
        dimensions = b.dimensions;
        massCenter = b.massCenter;
        boundingBoxMin = b.boundingBoxMin;
        boundingBoxMax = b.boundingBoxMax;
        centroid = b.centroid;
        mesh = b.mesh;
        minX = b.minX;
        minY = b.minY;
        minZ = b.minZ;
        maxX = b.maxX;
        maxY = b.maxY;
        maxZ = b.maxZ;

        id = -1;
        deltaVolume = 0.0f;
        markedForDeletion = false;
        framesLeft = 0;
    }




    int getLowestError() {
        int best=-1;
        float best_v=99999.0f;
        for( unsigned int i=0; i<error.size(); i++ ) {
            if( error[i] < best_v ) {
                best = i;
                best_v = error[i];
            }
        }
        return best;
    }


};


#endif


