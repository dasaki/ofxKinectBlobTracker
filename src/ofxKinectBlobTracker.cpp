/*
* ofxCvBlobTracker.cpp
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

#include "ofxKinectBlobTracker.h"


ofxKinectBlobTracker::ofxKinectBlobTracker() {
    listener = NULL;
	currentID = 1;
	extraIDs = 0;
	rejectDistThresh = DEFAULT_REJECT_DISTANCE_THRESHOLD;
	minDisplacementThresh = DEFAULT_MINIMUM_DISPLACEMENT_THRESHOLD;
	ghost_frames = DEFAULT_GHOST_FRAMES;
}


void ofxKinectBlobTracker::setListener( ofxKinectBlobListener* _listener ) {
    listener = _listener;
}


int ofxKinectBlobTracker::findOrder( int id ) {
    // This is a bit inefficient but ok when
    // assuming low numbers of blobs
    // a better way would be to use a hash table
    int count = 0;
    for( unsigned int i=0; i<blobs.size(); i++ ) {
        if( blobs[i].id < id ) {
            count++;
        }
    }
    return count;
}


int ofxKinectBlobTracker::getIndexById( int id ) {


    // This is a bit inefficient but ok when
    // assuming low numbers of blobs
    // a better way would be to use a hash table
    for(unsigned int i=0; i<blobs.size(); i++ ) {
        if( blobs[i].id == id ) {
            return i;
        }
    }
	return -1;
}

void ofxKinectBlobTracker::getTrajectoryById( int id, vector <ofVec3f> &trajectory) {



    for (unsigned int i=0; i<MAX_HISTORY_LEN; i++ ) {
        unsigned int j;
        for( j=0; j < history[i].size(); j++ ) {
            if( history[i][j].id == id ) {
                trajectory.push_back(history[i][j].massCenter);
                break;
            }
        }
        // blob is younger than whole history
        if (j>= history[i].size()) break;
    }
}


void ofxKinectBlobTracker::draw( ) {


//	ofEnableAlphaBlending();
//	ofSetColor( 255,0,200,100 );
    glPushMatrix();
    ofSetHexColor(0xffffff);
	for( unsigned int i=0; i<blobs.size(); i++ ) {
		/*glBegin(GL_LINE);
		 for( unsigned int j=0; j < history[i].size(); j++ ) {
            glVertex3f( history[i][j].massCenter.x, history[i][j].massCenter.y, history[i][j].massCenter.z );
        }
*/     /* ofTranslate(blobs[i].massCenter);
        testFont.drawStringAsShapes(ofToString(blobs[i].id), 0, 0);
		glEnd();*/
	}

   /* ofSetHexColor( 0xffffff );
    for( int i=0; i<blobs.size(); i++ ) {
        ostringstream docstring;
        //docstring << blobs[i].id << endl;
        docstring << findOrder(blobs[i].id) << endl;
        ofDrawBitmapString( docstring.str(),
                            blobs[i].massCenter.x, blobs[i].massCenter.y );
    }*/
	glPopMatrix();
}






/**
* Assign ids to blobs and fire blob events.
* This method tracks by proximity and best fit.
*/
void ofxKinectBlobTracker::trackBlobs( const vector<ofxKinectBlob>& _blobs) {
	unsigned int i, j, k;

    // Push to history, clear
	history.push_back( blobs );
	if( history.size() > MAX_HISTORY_LEN ) {
		history.erase( history.begin() );
	}
	blobs.clear();

    // Load new blobs
	for( i=0; i<_blobs.size(); i++ ) {
		blobs.push_back( ofxKinectTrackedBlob(_blobs[i]) );
	}

	vector<ofxKinectTrackedBlob> *prev = &history[history.size()-1];

	unsigned int cursize = blobs.size();
	unsigned int prevsize = (*prev).size();


	// now figure out the 'error' (distance) to all blobs in the previous
    // frame. We are optimizing for the least change in distance.
    // While this works really well we could also optimize for lowest
    // deviation from predicted position, change in size etc...

	for( i=0; i<cursize; i++ ) {
		blobs[i].error.clear();
		blobs[i].closest.clear();


		for( j=0; j<prevsize; j++ ) {
            //calc error - distance to blob in prev frame
           float error = blobs[i].massCenter.distance((*prev)[j].massCenter);

			blobs[i].error.push_back( error );
			blobs[i].closest.push_back( j );
		}
	}

	// sort so we can make a list of the closest blobs in the previous frame..
	for( i=0; i<cursize; i++ ) {
		// Bubble sort closest.
		for( j=0; j<prevsize; j++ )	{
			for( k=0; k<prevsize-1-j; k++ )	{
				// ugly as hell, I know.
				if( blobs[i].error[blobs[i].closest[k+1]]
                    < blobs[i].error[blobs[i].closest[k]] ) {

                    int tmp = blobs[i].closest[k];  // swap
                    blobs[i].closest[k] = blobs[i].closest[k+1];
                    blobs[i].closest[k+1] = tmp;
				}
			}
		}
	}


	// Generate a matrix of all the possible choices.
	// Then we will calculate the errors for every possible match
    // and pick the matrix that has the lowest error.
    // This is an NP complete approach and exponentially increases in complexity
    // with the number of blobs. To remedy for each blob we will only
    // consider the 4 closest blobs of the previous frame.

	ids.clear();


	// collect id's..
	for( i=0; i<cursize; i++ ) {
		ids.push_back( -1 );
	}

	extraIDs = cursize - prevsize;
	if( extraIDs < 0 ) {
		extraIDs = 0;
    }
	matrix.clear();


	// FIXME: we could scale numcheck depending on how many blobs there are
	// if we are tracking a lot of blobs, we could check less..

	if( cursize <= 4 ) {
		numcheck = 4;
	} else if( cursize <= 6 ) {
		numcheck = 3;
	} else if( cursize <= 10 ) {
		numcheck = 2;
	} else {
		numcheck = 1;
    }

	if( prevsize < numcheck ) {
		numcheck = prevsize;
	}

	if( blobs.size() > 0 ) {
		permute(0);
    }


	unsigned int num_results = matrix.size();


	// loop through all the potential
    // ID configurations and find one with lowest error

	float best_error = 99999, error;
	int best_error_ndx = -1;

	for( j=0; j<num_results; j++ ) {
		error = 0;
		// get the error for each blob and sum
		for( i=0; i<cursize; i++ ) {
			//ofxCvTrackedBlob *f = 0;

			if( matrix[j][i] != -1 ) {
				error += blobs[i].error[matrix[j][i]];
			}
		}

		if( error < best_error)	{
			best_error = error;
			best_error_ndx = j;
		}
	}


	// now that we know the optimal configuration,
    // set the IDs and calculate some things..

	if( best_error_ndx != -1 ) {
		for( i=0; i<cursize; i++ ) {
			if( matrix[best_error_ndx][i] != -1 ) {
				blobs[i].id = (*prev)[matrix[best_error_ndx][i]].id;
			} else {
				blobs[i].id = -1;
            }

			if( blobs[i].id != -1 ) {
				ofxKinectTrackedBlob *oldblob = &(*prev)[matrix[best_error_ndx][i]];

				blobs[i].deltaLoc = blobs[i].massCenter - oldblob->massCenter;

				blobs[i].deltaVolume = blobs[i].volume - oldblob->volume;

				blobs[i].predictedPos = blobs[i].massCenter + blobs[i].deltaLoc;

				blobs[i].deltaLocTotal = oldblob->deltaLocTotal + blobs[i].deltaLoc;
			} else {
				blobs[i].deltaLoc = ofVec3f( 0.0f, 0.0f, 0.0f );
				blobs[i].deltaVolume = 0.0f;
				blobs[i].predictedPos = blobs[i].massCenter;
				blobs[i].deltaLocTotal = ofVec3f( 0.0f, 0.0f, 0.0f );
			}
		}
	}




    // fire events
    //

	// assign ID's for any blobs that are new this frame (ones that didn't get
	// matched up with a blob from the previous frame).
	for( i=0; i<cursize; i++ ) {
		if(blobs[i].id == -1)	{
			blobs[i].id = currentID;
			currentID ++;
			if( currentID >= 65535 ) {
				currentID = 0;
            }

			//doTouchEvent(blobs[i].getTouchData());
            doBlobOn( blobs[i] );
		} else {
            float totalLength = blobs[i].deltaLocTotal.length();
			if( totalLength >= minDisplacementThresh ) {
				//doUpdateEvent( blobs[i].getTouchData() );
                doBlobMoved( blobs[i] );
				blobs[i].deltaLocTotal = ofVec3f( 0.0f, 0.0f, 0.0f );
			}
		}
	}

	// if a blob disappeared this frame, send a blob off event
    // for each one in the last frame, see if it still exists in the new frame.
	for( i=0; i<prevsize; i++ ) {
		bool found = false;
		for( j=0; j<cursize; j++ ) {
			if( blobs[j].id == (*prev)[i].id ) {
				found = true;
				break;
			}
		}

		if( !found ) {
			if( ghost_frames == 0 )	{
				//doUntouchEvent((*prev)[i].getTouchData());
                doBlobOff( (*prev)[i] );

			} else if( (*prev)[i].markedForDeletion ) {
				(*prev)[i].framesLeft -= 1;
				if( (*prev)[i].framesLeft <= 0 ) {
					//doUntouchEvent( (*prev)[i].getTouchData() );
                    doBlobOff( (*prev)[i] );
				} else {
					blobs.push_back( (*prev)[i] );  // keep it around
                                                    // until framesleft = 0
                }
			} else {
				(*prev)[i].markedForDeletion = true;
				(*prev)[i].framesLeft = ghost_frames;
				blobs.push_back( (*prev)[i] );  // keep it around
                                                // until framesleft = 0
			}
		}
	}
}

ofVec3f ofxKinectBlobTracker::findVelocity(unsigned int index) {


	ofVec3f velocity = ofVec3f(0.0f,0.0f,0.0f);

     if ( (history.size() > 1) && (index < blobs.size())){
             ofVec3f oldMassCenter = ofVec3f(0.0f,0.0f,0.0f);

             int id = blobs[index].id;

            unsigned int i;
            for( i=0; i < history[1].size(); i++ ) {
                if( history[1][i].id == id ) {
                    oldMassCenter = history[1][i].massCenter;
                    break;
                }
            }
            if (i < history[1].size()) ofVec3f velocity = blobs[index].massCenter - oldMassCenter;
     }
    return velocity;
}

ofVec3f ofxKinectBlobTracker::findVelocityById(int id) {


	ofVec3f velocity = ofVec3f(0.0f,0.0f,0.0f);

     if ( (history.size() > 1) && (id <= currentID)){
         int index = getIndexById(id);
         if (id >= 0) {
             ofVec3f oldMassCenter = ofVec3f(0.0f,0.0f,0.0f);

            unsigned int i;
            for( i=0; i < history[1].size(); i++ ) {
                if( history[1][i].id == id ) {
                    oldMassCenter = history[1][i].massCenter;
                    break;
                }
            }
            if (i < history[1].size()) ofVec3f velocity = blobs[index].massCenter - oldMassCenter;
         }
     }



    return velocity;
}


// Delegate to Callbacks
//
//
void ofxKinectBlobTracker::doBlobOn( const ofxKinectTrackedBlob& b ) {
    if( listener != NULL ) {
        listener->blobOn( b.massCenter, b.id, findOrder(b.id) );
    } else {
        cout << "doBlobOn() event for blob: " << b.id << endl;
    }
}
void ofxKinectBlobTracker::doBlobMoved( const ofxKinectTrackedBlob& b ) {
    if( listener != NULL ) {
        listener->blobMoved( b.massCenter, b.id, findOrder(b.id) );
    } else {
        cout << "doBlobMoved() event for blob: " << b.id << endl;
    }
}
void ofxKinectBlobTracker::doBlobOff( const ofxKinectTrackedBlob& b ) {
    if( listener != NULL ) {
        listener->blobOff( b.massCenter, b.id, findOrder(b.id) );
    } else {
        cout << "doBlobOff() event for blob: " << b.id << endl;
    }
}





// Helper Methods
//
//
inline void ofxKinectBlobTracker::permute( unsigned int start ) {
    if( start == ids.size() ) {
        //for( int i=0; i<start; i++)
        //{
        //printf("%d, ", ids[i]);
        //}
        //printf("--------\n");
        matrix.push_back( ids );

    } else {
        unsigned int numchecked = 0;

        for( unsigned int i=0; i<blobs[start].closest.size(); i++ ) {
            if( blobs[start].error[blobs[start].closest[i]]
                > rejectDistThresh ) {
                break;
            }

            ids[start] = blobs[start].closest[i];
            if( checkValid(start) ) {
                permute( start+1 );
                numchecked++;
            }

            if( numchecked >= numcheck ) {
                break;
            }
        }

        if( extraIDs > 0 ) {
            ids[start] = -1;		// new ID
            if( checkValidNew(start) ) {
                permute(start+1);
            }
        }
    }
}

inline bool ofxKinectBlobTracker::checkValidNew( int start ) {
	int newidcount = 0;
	newidcount ++;
	for( int i=0; i<start; i++ ) {
        if(ids[i] == -1) {
            newidcount ++;
        }
	}

	if( newidcount > extraIDs ) {
        return false;
    }

	return true;
}

inline bool ofxKinectBlobTracker::checkValid( int start ) {
	for(int i=0; i<start; i++) {
        // check to see whether this ID exists already
        if(ids[i] == ids[start]) {
            return false;
        }
	}

    return true;
}




