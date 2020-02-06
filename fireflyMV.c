#include "grab_fireflyMV/camera_select.c"
#include "grab_fireflyMV/udp.c"
#include "grab_fireflyMV/show_image.c"
#include "grab_fireflyMV/image_proc.c"
#include "grab_fireflyMV/init.c"
#include "grab_fireflyMV/fusion.c"

using namespace std;

int main(int argc, char *argv[]){

    init();

    while(play){

        //capture and process frames, uses one thread for each camera
        captureAndProcess();

        // send marker data
        for(int i=0; i < 100; i++){

        	if(markerFound[i]){

                // Marker with id i has been found by at least one camera, start averaging
                computeAverage(i);

                sendCoords = avgCoords;
                sendAngles = avgAngles;
            	
            	/*if(detectingMotion){

                    detectFirstMotion(i);

            	} else {

                    // camera no check and transition mitigation
                    checkState();

                    // angle check mitigation
                    checkAngleAndDist(i);

            	} // if/else detectingMotion */

                UDPSend(i, sendCoords, sendAngles);

	        } // if marker found 

        } // for i

	    showFrames();
        
    }

    /** Cleanup **/

    for( int i = 0; i < (int)list -> num; i++){

        cameraCleanup(cameras[i]);

    }

    dc1394_free (d);
    dc1394_camera_free_list (list);
    return 0;

}