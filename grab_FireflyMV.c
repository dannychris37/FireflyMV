#include "grab_fireflyMV/camera_select.c"
#include "grab_fireflyMV/udp.c"
#include "grab_fireflyMV/show_image.c"
#include "grab_fireflyMV/camera_control.c"
#include "grab_fireflyMV/image_proc.c"

using namespace std;


int main(int argc, char *argv[]){

    /** Add cameras that are to be used to cameraID vector **/
    addUsedCams();

	/** Setup UDP **/
    UDPset();

    FILE* imagefile;
    unsigned int width, height;
    
    /** Camera initialization  **/

    std::vector<dc1394camera_t*> cameras;

    // context in which the library can be used
    dc1394_t* d;

    // list of cameras to be located by dc1394_camera_enumerate
    dc1394camera_list_t * list;

    // enum with error macros to be used for error reporting
    dc1394error_t err;

    // initialize context
   	d = dc1394_new ();
	if (!d) return 1;

	// locate cameras
	err = dc1394_camera_enumerate (d, &list);
	DC1394_ERR_RTN(err, "Failed to enumerate cameras");

    // no cameras in the list
    if (list->num == 0) {

        dc1394_log_error("No cameras found");
        return 1;

    }

    cameras.resize(list->num);
    frames.resize(list->num);

    // check if provided GUIDs match located camera ids
    // and initialize a camera structure for each camera 
    for( int i = 0; i < (int)list -> num; i++){

    	// camera increment
        int j=0; 

        while (j<16){

            if ( cameraID.size() == (int)list -> num){

                if (cameraID[i] == list -> ids[j].guid) break;
                else j++;

            }
            else{

            	// no. of GUIDs provided is smaller than
            	// ammount of detected cameras
                std::cout << "Not enough GUID's to verify camera order" << std::endl;
                return 1;
            }
    	}

    	// initialize camera structure
		cameras[i] = dc1394_camera_new (d, list->ids[j].guid);

 		if (!cameras[i]) {
    		
    		dc1394_log_error("Failed to initialize camera with guid %llx", list->ids[j].guid);
    		
    		return 1;
 		}

 		// initialize physical cameras: operation mode, iso speed, image size etc.
		initializeCamera(cameras[i]);
        std::cout << "Initialised " << i+1 << " cameras" << std::endl;
                
    }

    /** Read Aruco Files (for camera calibration?) **/

    readArucoFiles(list->num);

    /** Runtime loop **/
    
    while(play){

    		// reinitialize sending data
    		// assigns 0's to sent_data vector
            sent_data.assign(sent_data.size(), 0); 
        
        	// capture frames from each camera
            for(int i = 0; i < (int)list -> num; i++){

                cameraCaptureSingle(cameras[i], i);

            }

            // show captured frames
            cv::Mat combImg = makeCombined(frames, 700, 2);
            cv::imshow("TruckLabImgs", combImg);
            keyPress(cameras, list);
    }

    /** Cleanup **/

    for( int i = 0; i < (int)list -> num; i++){

            cameraCleanup(cameras[i]);

    }

    dc1394_free (d);
    dc1394_camera_free_list (list);
    return 0;

}