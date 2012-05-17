#ifndef OF_KINECT_TRACKING_CONSTANTS_H
#define OF_KINECT_TRACKING_CONSTANTS_H

#include <iostream>
#include "ofxKinectBlob.h"
#include "ofxKinectTrackedBlob.h"

#define  MAX_HISTORY_LEN 16

//#define  MAX_NUM_CONTOURS_TO_FIND   128  // alther this if you think you will
                                         // be looking for more....
//#define  MAX_CONTOUR_LENGTH        1024  // alther this if you think your
                                         // contours will be longer than this
#define DEFAULT_REJECT_DISTANCE_THRESHOLD        0.3f
#define DEFAULT_MINIMUM_DISPLACEMENT_THRESHOLD   0.01f

#define DEFAULT_GHOST_FRAMES 4

class ofxKinectBlobListener {
  public:

    virtual void blobOn( ofVec3f centroid, int id, int order ) = 0;
    virtual void blobMoved( ofVec3f centroid, int id, int order ) = 0;
    virtual void blobOff( ofVec3f centroid, int id, int order ) = 0;

};


#endif
