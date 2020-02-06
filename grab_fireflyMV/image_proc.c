#include "fireflyMV.h"

/**  Convert in-image timestamp to seconds **/

inline double imageTimeStampToSeconds(unsigned int uiRawTimestamp) {
   // 32 bits in total
   // [7 bits seconds] - [13 bits cycle count] - [12 bits cycle offset]
   // max values [127] - [7999] - [3071]
   // cycle timer runs at 8kHz (hence the devide by 8000.0f)
   // 3072 cycle ticks in a cycle
    
   int nSecond      = (uiRawTimestamp >> 25) & 0x7F;   // get rid of cycle and offset
   int nCycleCount  = (uiRawTimestamp >> 12) & 0x1FFF; // get rid of seconds and offset
   int nCycleOffset = (uiRawTimestamp >>  0) & 0xFFF;  // get seconds and cycle
   return (double)nSecond + ( ((double)nCycleCount + ((double)nCycleOffset/3072.0)) / 8000.0 );   
}

void readArucoFiles(int total_camera){

    camMatrix.resize(total_camera);
    distCoeffs.resize(total_camera);

    // reading all files for aruco
    bool read = readDetectorParameters("detector_params.yml", detectorParams);

    if(!read) {

       	std::cerr << "Invalid detector parameters file" << std::endl;
        play = false;
    	 //return 0;

    }

    for (int i = 0; i < total_camera; i++){

        char filename[50];
        sprintf(filename,"calibration_files/camera_%d_checker.yml",i);
        bool readOk = readCameraParameters(filename, camMatrix[i], distCoeffs[i]);

        if(!readOk) {

            std::cerr << "Invalid camera file for camera number: "<< i + 1 << std::endl;
            play =false;
            //return 0;

        }
    }
}

void getEulerAngles(
	cv::Mat &rotCamerMatrix,
	cv::Vec3d &eulerAngles
	) {

    cv::Mat 
    	cameraMatrix,
    	rotMatrix,
    	transVect,
    	rotMatrixX,
    	rotMatrixY,
    	rotMatrixZ;

    double* _r = rotCamerMatrix.ptr<double>();

    double projMatrix[12] = 
    	{ 	_r[0], _r[1], _r[2], 0,
			_r[3], _r[4], _r[5], 0,
            _r[6], _r[7], _r[8], 0
        };

    cv::decomposeProjectionMatrix( 
    	cv::Mat(3,4,CV_64FC1,projMatrix),
        cameraMatrix,
        rotMatrix,
        transVect,
        rotMatrixX,
        rotMatrixY,
        rotMatrixZ,
        eulerAngles
    );
}

void makeSense(cv::Vec3d tvec, cv::Vec3d rvec, int markerID, int camera_no){
    
    /** Fixed marker IDs **/
    if(markerID < 10) {
        
        cv::Mat rotMatrix;

        // converts a rotation matrix to a 
        // rotation vector or viceversa
		cv::Rodrigues(rvec, rotMatrix);

		// transpose rotation matrix
        transpose(rotMatrix, f_rotMat[camera_no]);
        f_tvec[camera_no] = -1 * tvec;
        f_markerID[camera_no] = markerID;

    }

    else{
        
        // only moving markers
        if (markerID > 10){  
           
           	// if all x pointing in same direction
            cv::Matx33d rotMattoe0( 
            	0.0, -1.0, 0.0,
            	1.0, 0.0, 0.0,
            	0.0, 0.0, 1.0
            );

            // find out absolute coordinates of moving markers
            cv::Vec3d reading = 
            	(rotMattoe0 * (// multiply with rotation matrix few lines above (3 x 3)
                    f_rotMat[camera_no] * // (3 x 3)
                        (tvec + f_tvec[camera_no])
                    )
                ) + 
                // add to fixed marker absolute position
            	transtoe0[f_markerID[camera_no]];

            cv::Mat rotationMatrix;
            cv::Rodrigues(rvec, rotationMatrix);
            cv::Vec3d angle_rot;

            getEulerAngles(rotationMatrix, angle_rot);

            
            // mutex for shared DS and printing
            std::unique_lock<std::mutex> lck_pose_print(mtx_pose_print);
            cnd_var_pose_print.wait(lck_pose_print, []{return can_print_poses;});

            dataToSend[markerID][camera_no].coords = reading;
            dataToSend[markerID][camera_no].angles = angle_rot;
            dataToSend[markerID][camera_no].fixedMarker = f_markerID[camera_no];
            dataToSend[markerID][camera_no].valuesStored = true;
            markerFound[markerID] = true;

            cnd_var_pose_print.notify_one();

        }
    }
}

void arucoPipeline(cv::Mat img, int camera_no) {
    
    // reset for each frame
    f_markerID[camera_no] = 0; 
    std::vector<int> markerIds;
    std::vector< std::vector<cv::Point2f> > markerCorners,rejectedCandidates;
    std::vector< cv::Vec3d >  rvecs, tvecs;
    
    // fixed and moving markers have different dictionaries hence the search algorithm is changed 
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // ArUco module function that performs marker detection
    cv::aruco::detectMarkers(
    	img,				// image where markers are detected
    	dictionary,			// dictionary used
    	markerCorners,		// where to store marker corners
    	markerIds,			// where to store marker ID's
    	detectorParams,		// params that can be customized during detection
    	rejectedCandidates 	// where to store rejected candidates
	);

    // ArUco module function that draws detected markers on input image
    cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);

    // rejected candidates
    // cv::aruco::drawDetectedMarkers(img,rejectedCandidates , cv::noArray(), cv::Scalar(100, 0, 255)); 
    
    // if markers have same dictionary 
    if (markerIds.size() > 0) {

		// sending on fixed marker first
		bool found_fixedM = 0;

		for(unsigned int i = 0; i < markerIds.size(); i++){ 
                
            std::vector< std::vector< cv::Point2f > > single_markerCorner;
            std::vector<cv::Vec3d> single_rvec, single_tvec;
            single_markerCorner.resize(1);
            single_markerCorner[0] = markerCorners[i];

            //std::cout << "processed markercorner size"<< single_markerCorner.size()<<std::endl;
                
            /** Fixed marker ID **/
            if (markerIds[i] < 10){

            		// estimate pose
                    cv::aruco::estimatePoseSingleMarkers(
                        single_markerCorner,       // marker corners
                    	markerLength_fixed,        // size of marker
                    	camMatrix[camera_no],      // camera calibration parameter (known a priori)
                    	distCoeffs[camera_no],     // camera calibration parameter (known a priori)
                    	single_rvec,               // rotation vector of fixed marker
                    	single_tvec                // translation vector of fixed marker
                    );

                    //std::cout<<"fixed markers:"<<markerIds[i]<<std::endl;

                    // draws X, Y, Z axes
                    cv::aruco::drawAxis(
                    	img, 
                    	camMatrix[camera_no], 
                    	distCoeffs[camera_no], 
                    	single_rvec[0], 
                    	single_tvec[0],
    		            markerLength_fixed*0.5f
    		        );

                    makeSense(single_tvec[0], single_rvec[0], markerIds[i], camera_no);

                    found_fixedM = 1;

            }
      }

    /** Send moving markers **/
	if(found_fixedM == 1){

      	for(unsigned int i = 0; i < markerIds.size(); i++){

            std::vector< std::vector<cv::Point2f> > single_markerCorner;
            std::vector<cv::Vec3d> single_rvec,single_tvec;
            single_markerCorner.resize(1);
            single_markerCorner[0] = markerCorners[i];
            
            /** Moving marker ID **/
            if(markerIds[i] > 10){

                    cv::aruco::estimatePoseSingleMarkers( 
                        single_markerCorner,       // marker corners
                    	markerLength_moving,       // size of marker
                    	camMatrix[camera_no],      // camera calibration parameter (known a priori)
                    	distCoeffs[camera_no],     // camera calibration parameter (known a priori)
                    	single_rvec,               // rotation vector of moving marker
                        single_tvec                // translation vector of moving marker
                    );

                    //verify
                    //std::cout<<"moving markers:"<<markerIds[i]<<std::endl;
                    //std::cout<<"moving markers:"<<single_rvec.size()<<std::endl;

                    //find center
                    //if(print_flag && markerIds[i]==13) cout<<"Camera "<<camera_no<<" tvec: "<<single_tvec[0]<<endl;

                    cv::aruco::drawAxis(
                    	img, 
                    	camMatrix[camera_no], 
                    	distCoeffs[camera_no], 
                    	single_rvec[0], 
                    	single_tvec[0],
                    	markerLength_moving*0.5f
                    );

                    makeSense(single_tvec[0], single_rvec[0], markerIds[i], camera_no);

            }
      	}
	}

    //rvecs[i]=single_rvec[0];
    //tvecs[i]=single_tvec[0];
    //makeSense(single_tvec[0],single_rvec[0],markerIds[i]);

    }
}

dc1394error_t cameraCaptureSingle(
	dc1394camera_t* camera,
	int camera_no
) {

    if(MEAS_WAIT){

        clock_gettime(CLOCK_MONOTONIC, &start_wait[camera_no]);

    }

    dc1394error_t err;
    dc1394video_frame_t* frame = NULL;
    dc1394video_frame_t frame_buffer;

    unsigned int stamp;
    unsigned char* pStamp = (unsigned char*) &stamp;

    cv::Mat BImage = cv::Mat( 480, 640, CV_8UC1 );
    cv::Mat dispImage = cv::Mat( 480, 640, CV_8UC3 );
    cv::Mat finalImage = cv::Mat( 480, 640, CV_8UC3 );
    
    /** Capture one frame **/

    /** From libc1394 code
     * Captures a video frame. The returned struct contains the image buffer, among others. 
     * This image buffer SHALL NOT be freed, as it represents an area
     * in the memory that belongs to the system.

        dc1394error_t dc1394_capture_dequeue(
            dc1394camera_t * camera, 
            dc1394capture_policy_t policy, 
            dc1394video_frame_t **frame
        );

     */

    err = dc1394_capture_dequeue(
    	camera, 
    	DC1394_CAPTURE_POLICY_WAIT, 
    	&frame
    );

    /*if(camera_no == 7){
        clock_gettime(CLOCK_MONOTONIC, &stop_wait7);

        delta_wait7 = ( stop_wait7.tv_sec - start_wait[camera_no].tv_sec )
                 + (double)( stop_wait7.tv_nsec - start_wait[camera_no].tv_nsec )
                   / (double)MILLION;

        std::cout << "Cam " << camera_no << " frame waiting time (seven): " << delta_wait7 << "\n";
    }*/

    if(MEAS_WAIT){

        clock_gettime(CLOCK_MONOTONIC, &stop_wait[camera_no]);

        delta_wait[camera_no] = ( stop_wait[camera_no].tv_sec - start_wait[camera_no].tv_sec )
                 + (double)( stop_wait[camera_no].tv_nsec - start_wait[camera_no].tv_nsec )
                   / (double)MILLION;
        
        // wait until main allows for printing or previous thread notified
        std::unique_lock<std::mutex> lck_wait(mtx_wait);
        cnd_var_wait.wait(lck_wait, []{return can_print_wait_times;});

        if(print_flag)
            std::cout << "Cam " << camera_no << " frame waiting time: " << delta_wait[camera_no] << "\n";

        // increment counter
        print_wait_cnt++;

        // notify next thread or main
        cnd_var_wait.notify_one();

        //std::cout << "Cam " << camera_no << " thread is done wait time printing\n";

    }

    if(MEAS_PROC){

        clock_gettime(CLOCK_MONOTONIC, &start_proc[camera_no]);

    }

    DC1394_ERR_CLN_RTN(
    	err, 
    	cleanup_and_exit(camera),
    	"Could not capture a frame\n"
    );

    /* See libc1394 code for what is inside a the dc1394video_frame_t data structure */
    if(camera_no == FRAME_INFO_CAM && SHOW_FRAME_INFO){
        
        std::cout << "Frame received from camera " << camera_no + 1 << std::endl;
        
        /* the number of bytes used for the image (image data only, no padding) */
        std::cout << "Image size: " << frame -> size[0] << "x" << frame -> size[1] << std::endl;
        
        /* the color coding used. This field is valid for all video modes. */
        std::cout << "Color coding: " << frame -> color_coding << std::endl;
        
        /* the number of bits per pixel. The number of grayscale levels is 2^(this_no).
        This is independent from the colour coding */
        std::cout << "Data depth: " << frame -> data_depth << std::endl;
        
        /* the video mode used for capturing this frame */
        std::cout << "Video mode: " << frame -> video_mode << std::endl;
        
        /* the number of bytes used for the image (image data only, no padding) */
        std::cout << "Image bytes: " << frame -> image_bytes << "B" << std::endl;
        
        /* the number of extra bytes, i.e. total_bytes-image_bytes.  */
        std::cout << "Padding bytes: " << frame -> padding_bytes << "B" << std::endl;

        /* the total size of the frame buffer in bytes. May include packet-
        multiple padding and intentional padding (vendor specific) */
        std::cout << "Total bytes: " << frame -> total_bytes << "B" << std::endl;
        
        /* amount of memory allocated in for the *image field. */
        std::cout << "Allocated image bytes: " << frame -> allocated_image_bytes << "B" << std::endl;
        
        /* the size of a packet in bytes. (IIDC data) */
        std::cout << "Packet size: " << frame -> packet_size << "B" << std::endl;
        
        /* the number of packets per frame. (IIDC data) */
        std::cout << "Packets per frame: " << frame -> packets_per_frame << std::endl;
    }
    
    /** Check if frame is corrupt **/

    if( dc1394_capture_is_frame_corrupt( camera, frame) )
        printf("\n[DEBUG] frame is corrupt!\n"); 
     
    memcpy( &frame_buffer, frame, sizeof(dc1394video_frame_t) );
    
    /* strictly little endian... */

    pStamp[ 0 ] = frame_buffer.image[3];
    pStamp[ 1 ] = frame_buffer.image[2];
    pStamp[ 2 ] = frame_buffer.image[1];
    pStamp[ 3 ] = frame_buffer.image[0];
    
    /** Copy pointer to opencv **/         
    
    BImage.data = frame_buffer.image;
    
    /** Convert to color image **/

    // OpenCV function. Converts and image from one color space to
    // another. cvtColor(src, dst, code - color conversion code)
    cvtColor( BImage, dispImage, CV_BayerRG2RGB );
    
    // So Deep copy
    finalImage = dispImage.clone();  

    /** After final image is obtained, run it through the ArUco
    	detection pipeline that detects and extracts poses of
    	static and moving markers **/
    arucoPipeline(finalImage, camera_no);

    // blocks if I remove this not sure why
    if(true){

        //std::cout << "Cam " << camera_no << " thread has reached pose count\n";
        std::unique_lock<std::mutex> lck_pose_cnt(mtx_pose_cnt);
        cnd_var_pose_cnt.wait(lck_pose_cnt, []{return can_print_poses;});

        pose_cnt++;

        cnd_var_pose_cnt.notify_one();

        //std::cout << "Cam " << camera_no << " incremented pose estimation count\n";

    }
    
    cv::Mat undistImg;

    // Transforms an image to compensate for lens distortion. 
    cv::undistort(
    	finalImage,
    	undistImg,
    	camMatrix[camera_no], 
    	distCoeffs[camera_no]
	);
    
    // Add undistorted image to vector of processed frames
    frames[camera_no] = undistImg;
    
    // releases DMA buffer so that memory can be reused
    // this has to follow a dma_capture function
    dc1394_capture_enqueue(camera, frame);

    if(MEAS_PROC){

        clock_gettime(CLOCK_MONOTONIC, &stop_proc[camera_no]);

        delta_proc[camera_no] = ( stop_proc[camera_no].tv_sec - start_proc[camera_no].tv_sec )
             + (double)( stop_proc[camera_no].tv_nsec - start_proc[camera_no].tv_nsec )
               / (double)MILLION;

        //std::cout << "Cam " << camera_no << " thread has reached processing time printing\n";

        // wait until main allows for printing
        std::unique_lock<std::mutex> lck_proc(mtx_proc);
        cnd_var_proc.wait(lck_proc, []{return can_print_proc_times;});

        if(print_flag)
            std::cout << "Cam " << camera_no << " frame processing time: " << delta_proc[camera_no] << "\n";

        // increment counter
        print_proc_cnt++;

        //lck.unlock();
        cnd_var_proc.notify_one();

    }

}

void captureAndProcess(){

    std::thread t[(int)list -> num];

    if(print_flag)
    std::cout << "---------------------------------------------------\n\n";

    // if measure while flag is set
    // start_while and other timespec vars are defined in firefly.h
    if(MEAS_WHILE){

        clock_gettime(CLOCK_MONOTONIC, &start_while);

    }

    // reset valuesStored and markerFound flags
    for(int i = 0; i < 100; i++){

        markerFound[i] = false;

        for(int j = 0; j < 8; j++){
            dataToSend[i][j].valuesStored = false;
        }
    }

    // capture frames from each camera
    for(int i = 0; i < (int)list -> num; i++){

        // waiting for and processing frame times are measured inside cameraCaptureSingle
        
        t[i] = std::thread(cameraCaptureSingle, cameras[i], i);

    }

    /** COORDINATE WAIT TIME PRINTING **/

    if(print_flag)
        std::cout << "--- WAITING FOR FRAMES ---\n\n";

    // enable wait time printing and notify any waiting thread
    can_print_wait_times = true;
    cnd_var_wait.notify_one();

    // wait until frames arrive and waiting times are printed
    std::unique_lock<std::mutex> lck_wait(mtx_wait);
    cnd_var_wait.wait(lck_wait, []{return print_wait_cnt == (int)list -> num;});
    //std::cout << "print_wait_cnt " << print_wait_cnt << std::endl;

    /** COORDINATE POSE PRINTING **/        

    if(print_flag)
        std::cout << "\n--- PROCESSING FRAMES ---\n\n";

    // enable pose printing
    can_print_poses = true;
    cnd_var_pose_print.notify_one();
    cnd_var_pose_cnt.notify_one();

    // wait until frames are processed
    std::unique_lock<std::mutex> lck_pose_cnt(mtx_pose_cnt);
    cnd_var_pose_cnt.wait(lck_pose_cnt, []{return pose_cnt == (int)list -> num;});
    //std::cout << "pose_cnt " << pose_cnt << std::endl;

    /** COORDINATE PROC TIME PRINTING **/

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
  

    for(int i = 0; i < (int)list -> num; i++){

        // waiting for and processing frame times are measured inside cameraCaptureSingle
        t[i].join();

    }
}