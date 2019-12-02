#include "fireflymv.h"

// vector of all processed aruco frames
vector<Mat> frames;

// camera matrix and distance coefficients used in 
// readArucoFiles function
vector<Mat> camMatrix, distCoeffs;

// fixed marker rvec->rotmat,tvec (global declaration)
Matx33d f_rotMat;
Vec3d f_tvec;
int f_markerID;

/** Translations to e0 (ground) coordinate system **/

vector<Vec3d> transtoe0 {{0.19,0.055,0.0},     //0 fixed marker in dM
                                  {0.19,0.4113,0.0},    //1 fixed marker
                                  {0.268,0.0549,0.0},   //2 fixed marker
                                  {0.343,0.41,0.0},     //3 fixed marker
                                  {0.5269,0.055,0.0},   //4 fixed marker 
                                  {0.4865,0.4098,0.0},  //5 fixed marker
                                  {0.6559,0.0549,0.0},  //6 fixed marker
                                  {0.6544,0.41,0.0},    //7 fixed marker 
                                  {0.19,0.4113,0.0}     //8 fixed marker
};

vector<bool> sent_data{    
    0,  // markerID 51 
    0   // markerID 52  
};

// only for fixed markers
const float markerLength_fixed      = 0.0203;

// only for moving markers
const float markerLength_moving     = 0.013;//0.0097;

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

       	cerr << "Invalid detector parameters file" << endl;
        play = false;
    	 //return 0;

    }

    for (int i = 0; i < total_camera; i++){

        char filename[50];
        sprintf(filename,"calibration_files/camera_%d_checker.yml",i);
        bool readOk = readCameraParameters(filename, camMatrix[i], distCoeffs[i]);

        if(!readOk) {

            cerr << "Invalid camera file for camera number: "<< i + 1 << endl;
            play =false;
            //return 0;

        }
    }
}

void getEulerAngles(
	Mat &rotCamerMatrix,
	Vec3d &eulerAngles
	) {

    Mat 
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

    decomposeProjectionMatrix( 
    	Mat(3,4,CV_64FC1,projMatrix),
        cameraMatrix,
        rotMatrix,
        transVect,
        rotMatrixX,
        rotMatrixY,
        rotMatrixZ,
        eulerAngles
    );
}

void makeSense(Vec3d tvec,Vec3d rvec, int markerID, int camera_no){
    
    /** Fixed marker IDs **/
    if(markerID < 50) {
        
        Mat rotMatrix;

        // converts a rotation matrix to a 
        // rotation vector or viceversa
		Rodrigues(rvec, rotMatrix);

		// transpose rotation matrix
        transpose(rotMatrix, f_rotMat);
        f_tvec = -1 * tvec;
        f_markerID = markerID;

    }

    else{
        
        // only moving markers
        if (markerID > 50){  
           
           	// if all x pointing in same direction
            Matx33d rotMattoe0( 
            	0.0, -1.0, 0.0,
            	1.0, 0.0, 0.0,
            	0.0, 0.0, 1.0
            );

            // 
            Vec3d reading = 
            	(rotMattoe0 * (f_rotMat * (tvec + f_tvec))) + 
            	transtoe0[f_markerID];

            cout << "CAM: using fixed marker ID:" << f_markerID << endl;
            cout << "CAM: origin to truck :" << markerID << "\t" << reading << endl;
            
            Mat rotationMatrix;
            Rodrigues(rvec, rotationMatrix);
            Vec3d angle_rot;

            
            getEulerAngles(rotationMatrix, angle_rot);
            cout << "CAM: rotation angle(deg):" << "\t" << angle_rot << endl;
            
            if (sent_data[markerID-51] == 0){
                cout << "\nSEND: Cam "<<camera_no<<" first to find marker "<<markerID<<endl;
                cout << "SEND: Coordinates to send:\t" << reading << endl;
                cout << "SEND: Angles to send:\t\t" << angle_rot << endl;
            	// angles in degreee and x,y,z 
                UDPfarewell(markerID, reading, angle_rot);
                sent_data[markerID-51] = 1;

            }
            
            else{

                cout << "\nSKIP: Cam "<<camera_no<<" skipped: " << markerID << endl;

            }
        }
    }
}

void arucoPipeline(Mat img, int camera_no) {
    
    // reset for each frame
    f_markerID = 0; 
    vector<int> markerIds;
    vector< vector<Point2f> > markerCorners,rejectedCandidates;
    vector< Vec3d >  rvecs, tvecs;
    
    // fixed and moving markers have different dictionaries hence the search algorithm is changed 
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

    // ArUco module function that performs marker detection
    aruco::detectMarkers(
    	img,				// image where markers are detected
    	dictionary,			// dictionary used
    	markerCorners,		// where to store marker corners
    	markerIds,			// where to store marker ID's
    	detectorParams,		// params that can be customized during detection
    	rejectedCandidates 	// where to store rejected candidates
	);

    // ArUco module function that draws detected markers on input image
    aruco::drawDetectedMarkers(img,markerCorners,markerIds);

    // rejected candidates
    //aruco::drawDetectedMarkers(img,rejectedCandidates , noArray(), Scalar(100, 0, 255)); 
    
    // if markers have same dictionary 
    if (markerIds.size() > 0) {

		// sending on fixed marker first
		bool found_fixedM = 0;

		for(unsigned int i = 0; i < markerIds.size(); i++){ 
                
            vector< vector< Point2f > > single_markerCorner;
            vector<Vec3d> single_rvec, single_tvec;
            single_markerCorner.resize(1);
            single_markerCorner[0] = markerCorners[i];

            //cout << "processed markercorner size"<< single_markerCorner.size()<<endl;
                
            /** Fixed marker ID **/
            if (markerIds[i] < 50){

            		// estimate pose
                    aruco::estimatePoseSingleMarkers(single_markerCorner, 
                    	markerLength_fixed, 
                    	camMatrix[camera_no], 
                    	distCoeffs[camera_no], 
                    	single_rvec, 
                    	single_tvec
                    );

                    //cout<<"fixed markers:"<<markerIds[i]<<endl;

                    // draws X, Y, Z axes
                    aruco::drawAxis(
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

            vector< vector<Point2f> > single_markerCorner;
            vector<Vec3d> single_rvec,single_tvec;
            single_markerCorner.resize(1);
            single_markerCorner[0] = markerCorners[i];
            
            /** Moving marker ID **/
            if(markerIds[i] > 50){

                    aruco::estimatePoseSingleMarkers( single_markerCorner, 
                    	markerLength_moving, 
                    	camMatrix[camera_no], 
                    	distCoeffs[camera_no],  
                    	single_rvec,single_tvec
                    );

                    //verify
                    //cout<<"moving markers:"<<markerIds[i]<<endl;
                    //cout<<"moving markers:"<<single_rvec.size()<<endl;

                    aruco::drawAxis(
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

        clock_gettime(CLOCK_MONOTONIC, &start_wait);

    }

    dc1394error_t err;
    dc1394video_frame_t* frame = NULL;
    dc1394video_frame_t frame_buffer;

    unsigned int stamp;
    unsigned char* pStamp = (unsigned char*) &stamp;

    Mat BImage = Mat( 480, 640, CV_8UC1 );
    Mat dispImage = Mat( 480, 640, CV_8UC3 );
    Mat finalImage = Mat( 480, 640, CV_8UC3 );
    
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

    if(MEAS_WAIT){

        clock_gettime(CLOCK_MONOTONIC, &stop_wait);

        cout << "---------------------------------------------------" << endl;
        cout << "On camera no. " << camera_no << endl;

        delta_wait = ( stop_wait.tv_sec - start_wait.tv_sec )
                 + (double)( stop_wait.tv_nsec - start_wait.tv_nsec )
                   / (double)MILLION;

        cout << "frame waiting time: " << delta_wait << "\n";

    } else{

        cout << "---------------------------------------------------" << endl;
        cout << "On camera no. " << camera_no << endl;

    }

    if(MEAS_PROC){

        clock_gettime(CLOCK_MONOTONIC, &start_proc);

    }

    DC1394_ERR_CLN_RTN(
    	err, 
    	cleanup_and_exit(camera),
    	"Could not capture a frame\n"
    );

    /* See libc1394 code for what is inside a the dc1394video_frame_t data structure */
    if(camera_no == FRAME_INFO_CAM && SHOW_FRAME_INFO){
        
        cout << "Frame received from camera " << camera_no + 1 << endl;
        
        /* the number of bytes used for the image (image data only, no padding) */
        cout << "Image size: " << frame -> size[0] << "x" << frame -> size[1] << endl;
        
        /* the color coding used. This field is valid for all video modes. */
        cout << "Color coding: " << frame -> color_coding << endl;
        
        /* the number of bits per pixel. The number of grayscale levels is 2^(this_number).
        This is independent from the colour coding */
        cout << "Data depth: " << frame -> data_depth << endl;
        
        /* the video mode used for capturing this frame */
        cout << "Video mode: " << frame -> video_mode << endl;
        
        /* the number of bytes used for the image (image data only, no padding) */
        cout << "Image bytes: " << frame -> image_bytes << "B" << endl;
        
        /* the number of extra bytes, i.e. total_bytes-image_bytes.  */
        cout << "Padding bytes: " << frame -> padding_bytes << "B" << endl;

        /* the total size of the frame buffer in bytes. May include packet-
        multiple padding and intentional padding (vendor specific) */
        cout << "Total bytes: " << frame -> total_bytes << "B" << endl;
        
        /* amount of memory allocated in for the *image field. */
        cout << "Allocated image bytes: " << frame -> allocated_image_bytes << "B" << endl;
        
        /* the size of a packet in bytes. (IIDC data) */
        cout << "Packet size: " << frame -> packet_size << "B" << endl;
        
        /* the number of packets per frame. (IIDC data) */
        cout << "Packets per frame: " << frame -> packets_per_frame << endl;
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
    
    Mat undistImg;

    // Transforms an image to compensate for lens distortion. 
    undistort(
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

        clock_gettime(CLOCK_MONOTONIC, &stop_proc);

        delta_proc = ( stop_proc.tv_sec - start_proc.tv_sec )
             + (double)( stop_proc.tv_nsec - start_proc.tv_nsec )
               / (double)MILLION;

        cout << "frame processing time: " << delta_proc << "\n";

    }

}