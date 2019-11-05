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
		cameras[i] = dc1394_camera_new (d, list -> ids[j].guid);

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

    std::thread t[(int)list -> num];

    while(play){

    	std::cout << "---------------------------------------------------" << std::endl;

    	// if measure while flag is set
    	// start_while and other timespec vars are defined in firefly.h
    	if(MEAS_WHILE){

    		clock_gettime(CLOCK_MONOTONIC, &start_while);

    	}

		// resets processed flags vector
        for(int i=0; i < 100; i++){
            markerProcessed[i] = false;
        }

    	// capture frames from each camera
        for(int i = 0; i < (int)list -> num; i++){

        	// waiting for and processing frame times are measured inside cameraCaptureSingle
        	
            t[i] = std::thread(cameraCaptureSingle, cameras[i], i);

        }

        if(USE_MUTEX){

        	/** COORDINATE WAIT TIME PRINTING **/

	        std::cout << "--- WAITING FOR FRAMES ---\n";

	        // enable wait time printing and notify any waiting thread
	        can_print_wait_times = true;
	        cnd_var_wait.notify_one();

	        // wait until frames arrive and waiting times are printed
	        std::unique_lock<std::mutex> lck_wait(mtx_wait);
	        cnd_var_wait.wait(lck_wait, []{return print_wait_cnt == (int)list -> num;});
	        //std::cout << "print_wait_cnt " << print_wait_cnt << std::endl;

			/** COORDINATE POSE PRINTING **/        

	        std::cout << "--- PROCESSING FRAMES ---\n";

	        // enable pose printing
	        can_print_poses = true;
	        cnd_var_pose_print.notify_one();
	        cnd_var_pose_cnt.notify_one();

	        // wait until frames are processed
	        std::unique_lock<std::mutex> lck_pose_cnt(mtx_pose_cnt);
	        cnd_var_pose_cnt.wait(lck_pose_cnt, []{return pose_cnt == (int)list -> num;});
	        //std::cout << "pose_cnt " << pose_cnt << std::endl;

	        /** COORDINATE PROC TIME PRINTING **/

	        std::cout << "--- PROCESSING DONE ---\n";

	        // enable proc time printing
	        can_print_proc_times = true;
	        cnd_var_proc.notify_one();

	        // wait until processing times are printed
	        std::unique_lock<std::mutex> lck_proc(mtx_proc);
	        cnd_var_proc.wait(lck_proc, []{return print_proc_cnt == (int)list -> num;});
	        //std::cout << "print_proc_cnt " << print_proc_cnt << std::endl;

	        // reset for next loop
	        print_wait_cnt = 0;
	        pose_cnt = 0;
	        print_proc_cnt = 0;
	        can_print_wait_times = false;
	        can_print_poses = false; 
	        can_print_proc_times = false;

        }        

        for(int i = 0; i < (int)list -> num; i++){

        	// waiting for and processing frame times are measured inside cameraCaptureSingle
            t[i].join();

        }

	    if(MEAS_SHOW){

	    	 clock_gettime(CLOCK_MONOTONIC, &start_show);

	    }

        // show captured frames
        cv::Mat combImg = makeCombined(frames, 800, 2);
        cv::imshow("TruckLabImgs", combImg);
        keyPress(cameras, list);

        if(MEAS_WHILE || MEAS_SHOW){

        	clock_gettime(CLOCK_MONOTONIC, &stop_while);

        }

        if(MEAS_SHOW){

	        delta_show = ( stop_while.tv_sec - start_show.tv_sec )
	             + (double)( stop_while.tv_nsec - start_show.tv_nsec )
	               / (double)MILLION;

	    	std::cout << "Frame show time: " << delta_show << "\n";

		}


        if(MEAS_WHILE){

	        delta_while = ( stop_while.tv_sec - start_while.tv_sec )
	             + (double)( stop_while.tv_nsec - start_while.tv_nsec )
	               / (double)MILLION;

	    	std::cout << "While loop time: " << delta_while << "\n";

		}

		std::cout << "---------------------------------------------------\n\n\n";
        
    }

    /** Cleanup **/

    for( int i = 0; i < (int)list -> num; i++){

            cameraCleanup(cameras[i]);

    }

    dc1394_free (d);
    dc1394_camera_free_list (list);
    return 0;

}