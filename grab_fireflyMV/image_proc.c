#include "fireflymv.h"

// vector of all processed aruco frames
std::vector<cv::Mat> frames;

// camera matrix and distance coefficients used in 
// readArucoFiles function
std::vector<cv::Mat> camMatrix, distCoeffs;

// fixed marker rvec->rotmat,tvec (global declaration)
cv::Matx33d f_rotMat;
cv::Vec3d f_tvec;
int f_markerID;

/** Translations to e0 (ground) coordinate system **/

std::vector<cv::Vec3d> transtoe0 {{0.19,0.055,0.0},     //0 fixed marker in dM
                                  {0.19,0.4113,0.0},    //1 fixed marker
                                  {0.268,0.0549,0.0},   //2 fixed marker
                                  {0.343,0.41,0.0},     //3 fixed marker
                                  {0.5269,0.055,0.0},   //4 fixed marker 
                                  {0.4865,0.4098,0.0},  //5 fixed marker
                                  {0.6559,0.0549,0.0},  //6 fixed marker
                                  {0.6544,0.41,0.0},    //7 fixed marker 
                                  {0.19,0.4113,0.0}     //8 fixed marker
};

std::vector<bool> sent_data{    
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

void makeSense(cv::Vec3d tvec,cv::Vec3d rvec,int markerID){
    
    /** Fixed marker IDs **/
    if(markerID < 50) {
        
        cv::Mat rotMatrix;

        // converts a rotation matrix to a 
        // rotation vector or viceversa
		cv::Rodrigues(rvec, rotMatrix);

		// transpose rotation matrix
        transpose(rotMatrix, f_rotMat);
        f_tvec = -1 * tvec;
        f_markerID = markerID;

    }

    else{
        
        // only moving markers
        if (markerID > 50){  
           
           	// if all x pointing in same direction
            cv::Matx33d rotMattoe0( 
            	0.0, -1.0, 0.0,
            	1.0, 0.0, 0.0,
            	0.0, 0.0, 1.0
            );

            // 
            cv::Vec3d reading = 
            	(rotMattoe0 * (f_rotMat * (tvec + f_tvec))) + 
            	transtoe0[f_markerID];

            std::cerr << "using fixed marker ID:" << f_markerID << std::endl;
            std::cerr << "origin to truck :" << markerID << "\t" << reading << std::endl;
            
            cv::Mat rotationMatrix;
            cv::Rodrigues(rvec, rotationMatrix);
            cv::Vec3d angle_rot;

            
            getEulerAngles(rotationMatrix, angle_rot);
            std::cerr << "rotation angle(deg):" << angle_rot << std::endl;
            
            if (sent_data[markerID-51] == 0){

            	// angles in degreee and x,y,z 
                UDPfarewell(markerID, reading, angle_rot);
                sent_data[markerID-51] = 1;

            }
            
            else{

                std::cout << "skipped:" << markerID << std::endl;

            }
        }
    }
}

void arucoPipeline(cv::Mat img,int camera_number) {
    
    // reset for each frame
    f_markerID = 0; 
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
    cv::aruco::drawDetectedMarkers(img,markerCorners,markerIds);

    // rejected candidates
    cv::aruco::drawDetectedMarkers(img,rejectedCandidates , cv::noArray(), cv::Scalar(100, 0, 255)); 
    
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
            if (markerIds[i] < 50){

            		// estimate pose
                    cv::aruco::estimatePoseSingleMarkers(single_markerCorner, 
                    	markerLength_fixed, 
                    	camMatrix[camera_number], 
                    	distCoeffs[camera_number], 
                    	single_rvec, 
                    	single_tvec
                    );

                    //std::cout<<"fixed markers:"<<markerIds[i]<<std::endl;

                    // draws X, Y, Z axes
                    cv::aruco::drawAxis(
                    	img, 
                    	camMatrix[camera_number], 
                    	distCoeffs[camera_number], 
                    	single_rvec[0], 
                    	single_tvec[0],
    		            markerLength_fixed*0.5f
    		        );

                    makeSense(single_tvec[0],single_rvec[0],markerIds[i]);

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
            if(markerIds[i] > 50){

                    cv::aruco::estimatePoseSingleMarkers( single_markerCorner, 
                    	markerLength_moving, 
                    	camMatrix[camera_number], 
                    	distCoeffs[camera_number],  
                    	single_rvec,single_tvec
                    );

                    //verify
                    //std::cout<<"moving markers:"<<markerIds[i]<<std::endl;
                    //std::cout<<"moving markers:"<<single_rvec.size()<<std::endl;

                    cv::aruco::drawAxis(
                    	img, 
                    	camMatrix[camera_number], 
                    	distCoeffs[camera_number], 
                    	single_rvec[0], 
                    	single_tvec[0],
                    	markerLength_moving*0.5f
                    );

                    makeSense(single_tvec[0],single_rvec[0],markerIds[i]);

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

    dc1394error_t err;
    dc1394video_frame_t* frame = NULL;
    dc1394video_frame_t frame_buffer;

    unsigned int stamp;
    unsigned char* pStamp = (unsigned char*) &stamp;

    cv::Mat BImage = cv::Mat( 480, 640, CV_8UC1 );
    cv::Mat dispImage = cv::Mat( 480, 640, CV_8UC3 );
    cv::Mat finalImage = cv::Mat( 480, 640, CV_8UC3 );
    
    /** Capture one frame **/

    err = dc1394_capture_dequeue(
    	camera, 
    	DC1394_CAPTURE_POLICY_WAIT, 
    	&frame
    );
    DC1394_ERR_CLN_RTN(
    	err, 
    	cleanup_and_exit(camera),
    	"Could not capture a frame\n"
    );
    
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
}