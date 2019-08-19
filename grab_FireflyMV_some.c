#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <dc1394/dc1394.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/time.h>
#include <time.h>
#include <endian.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <csignal>
#include <string>
#include <vector>
#include <algorithm>

//sending data udp
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <ctime>


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/videoio/videoio.hpp"

#ifndef _WIN32
#include <unistd.h>
#endif

#define USE_CAM_1 0
#define USE_CAM_2 0
#define USE_CAM_3 1
#define USE_CAM_4 1
#define USE_CAM_5 0
#define USE_CAM_6 0
#define USE_CAM_7 0
#define USE_CAM_8 0

using namespace std;
std::vector<cv::Mat> frames; // vector of all processed aruco frames
std::vector<long int> cameraID;

struct cam_entry {
	char const *cam_name;
	long int cam_id;
};

struct cam_entry cams[] = {
	"CAM1", 49712223537630471,
	"CAM2", 49712223537053658,
	"CAM3", 49712223537053659,
	"CAM4", 49712223537053660,
	"CAM5", 49712223537630465,
	"CAM6", 49712223537630451,
	"CAM7", 49712223537630448,
	"CAM8", 49712223537630468,
	"NO_CAM", 0
};

string name_for_id(long int id)
{
    int i = 0;
    long int cam_id = cams[i].cam_id;
    while (cam_id) {
        if (cam_id == id)
            return cams[i].cam_name;
        cam_id = cams[++i].cam_id;
    }
    return 0;
}

void print_cams(const std::vector<long int>& vec)
{
	std::cout << "The following cameras are used: ";
    for (auto x: vec) {
         std::cout << name_for_id(x) << ' ';
    }
    std::cout << '\n';
}

// check
//std::vector<long int> cameraID {49712223537053658,49712223537630471}; // incorrect 
// direclty write to camera registers to enable HDR and 12-to-10 bit compounding
bool hd    = false; // more stable measurements
bool comp  = true;
int  ecount = 44e3;
bool play = true;
cv::Ptr<cv::aruco::DetectorParameters> detectorParams=cv::aruco::DetectorParameters::create();
std::vector<cv::Mat> camMatrix, distCoeffs;

//fixed marker rvec->rotmat,tvec (global declaration)
cv::Matx33d f_rotMat;
cv::Vec3d f_tvec;
int f_markerID;

//translations to e0 (ground) coordinate system
std::vector<cv::Vec3d> transtoe0 {{0.19,0.055,0.0}, //0 fixed marker in dM
                                  {0.19,0.4113,0.0}, //1 fixed marker
                                  {0.268,0.0549,0.0}, //2 fixed marker
                                  {0.343,0.41,0.0}, //3 fixed marker
                                  {0.5269,0.055,0.0}, //4 fixed marker 
                                  {0.4865,0.4098,0.0},  //5 fixed marker
                                  {0.6559,0.0549,0.0},  //6 fixed marker
                                  {0.6544,0.41,0.0},  //7 fixed marker 
                                  {0.19,0.4113,0.0} //8 fixed marker
};
std::vector<bool> sent_data{    0, //marker ID 51 
                                0  //markerID 52  
};
const float markerLength_fixed=0.0203;//only for fixed markers
const float markerLength_moving=0.013;//0.0097;//only for moving markers
/*-------------------------------UDP SEND --------------------------------------------------
--------------------------------------------------------------------------------------------*/
int port=3000;
const char *ip_send="169.254.19.64";//"192.168.7.50";
int socket_status,send_status;
struct sockaddr_in addr;
void UDPset(){
    addr.sin_family = AF_INET;
    socket_status= inet_aton(ip_send,&addr.sin_addr);
    if (socket_status==0){
        std::cout<<"error in inet_aton "<< std::endl;
        exit(1);


    }
    addr.sin_port=htons(port);
    socket_status = socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if (socket_status==-1){
        std::cout<<"error in socket creation "<< std::endl;
    }
}

void UDPfarewell(int markerID,cv::Vec3d data,cv::Vec3d rot){
    time_t t = std::time(0);
    double now = static_cast<double> (t);
    //std::cout<<"stamps:"<<now<< std::endl;
    
    cv::Vec<double,8> send_data{now,double(markerID),data[0],data[1],data[2],rot[0],rot[1],rot[2]}; 
    
    send_status= sendto(socket_status,&send_data,sizeof(send_data),0,(struct sockaddr *)&addr, sizeof(addr));
    if (send_status==-1){
        std::cout<< "error sending data"<<"\n error code: "<< send_status << endl;
    }
}
 cv::Mat makeCombined(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows) {
            int N = vecMat.size();
            nRows  = nRows > N ? N : nRows; 
            int edgeThickness = 10;
            int imagesPerRow = ceil(double(N) / nRows);
            int resizeHeight = floor(((floor(double(windowHeight - edgeThickness) / nRows)) ) - edgeThickness);
            int maxRowLength = 0;

            std::vector<int> resizeWidth;
            for (int i = 0; i < N;) {
                    int thisRowLen = 0;
                    for (int k = 0; k < imagesPerRow; k++) {
                            double aspectRatio = double(vecMat[i].cols) / vecMat[i].rows;
                            int temp = int( ceil(resizeHeight * aspectRatio));
                            resizeWidth.push_back(temp);
                            thisRowLen += temp;
                            if (++i == N) break;
                    }
                    if ((thisRowLen + edgeThickness * (imagesPerRow + 1)) > maxRowLength) {
                            maxRowLength = thisRowLen + edgeThickness * (imagesPerRow + 1);
                    }
            }
            int windowWidth = maxRowLength;
            cv::Mat combinedImage(windowHeight, windowWidth, CV_8UC3, cv::Scalar(0, 0, 0));
            
            for (int k = 0, i = 0; i < nRows; i++) {
                    int y = i * resizeHeight + (i + 1) * edgeThickness;
                    int x_end = edgeThickness;
                    for (int j = 0; j < imagesPerRow && k < N; k++, j++) {
                            int x = x_end;
                            cv::Rect roi(x, y, resizeWidth[k], resizeHeight);
                            cv::Size s = combinedImage(roi).size();
                            // change the number of channels to three
                            cv::Mat target_ROI(s,CV_8UC3);
                            
                            vecMat[k].copyTo(target_ROI);
                            cv::resize(target_ROI, target_ROI, s);
                            if (target_ROI.type() != combinedImage.type()) {
                                target_ROI.convertTo(target_ROI, combinedImage.type());
                            }
                            target_ROI.copyTo(combinedImage(roi));
                            x_end += resizeWidth[k] + edgeThickness;
                    }
            }
            return combinedImage;
    }
/*-----------------------------------------------------------------------
 *  Releases the cameras and exits
 *-----------------------------------------------------------------------*/
void cleanup_and_exit(dc1394camera_t *camera)
{
    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);
    exit(1);
}

/*-----------------------------------------------------------------------
 *  Convert in-image timestamp to seconds
 *-----------------------------------------------------------------------*/
inline double imageTimeStampToSeconds(unsigned int uiRawTimestamp)
{
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


dc1394error_t setLuminosity( dc1394camera_t *camera, int lum )
{ 
    // Create a variable to hold the register values
    dc1394error_t err;
    uint32_t ulValue = 0;
    
    err=dc1394_set_control_register( camera, 0x1A00, 0xB0 );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set Auto exposure pixel count\n");
       
    err=dc1394_get_control_register( camera, 0x1A04, &ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Get Auto exposure pixel count\n");
    printf("Get Auto exposure pixel count (before) %x\n", ulValue);
    
    err=dc1394_set_control_register( camera, 0x1A04, lum);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set Auto exposure pixel count\n");
    usleep(1e5);
    err=dc1394_get_control_register( camera, 0x1A04, &ulValue );
    printf("Set Auto exposure pixel count (after) %x\n", ulValue);                
}

// set 12-to-10 bit compounding
dc1394error_t setFrameFormat( dc1394camera_t *camera )
{
    // set byte alignment to little endian
    // point grey register IMAGE_DATA_FORMAT
    // bit [0] is MSB
    printf("Set to little endianess\n");
    dc1394_set_control_register( camera, 0x1048, 0x0 );
    usleep(1e5);
       
    // set bit 31 to 1 to enable in-image timestamps
    // point grey register FRAME_INFO
    // bit [0] is MSB
    uint32_t ulValue = 0;
    dc1394_get_control_register( camera, 0x12F8, &ulValue );
    printf("Frame info (before) 0x%x\n",ulValue);
    ulValue = ulValue | 0x1;    
    dc1394_set_control_register( camera, 0x12F8, ulValue );
    dc1394_get_control_register( camera, 0x12F8, &ulValue );
    printf("Frame info (after) 0x%x\n",ulValue);
}



// set 12-to-10 bit compounding
dc1394error_t setCompounding( dc1394camera_t *camera, int on )
{
    // Create a variable to hold the register values
    dc1394error_t err;
    uint32_t ulValue = 0;

    // Set 0x1C (ADC mode control) to be the sensor register to be written to
    err=dc1394_set_control_register( camera, 0x1A00, 0x1C );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set control register to ADC control\n");
    
    // Set bit 9 and bit 1 of the register to 1 to enable compounding mode
    err=dc1394_get_control_register( camera, 0x1A04, &ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Get HDR register\n");
    printf("Compound register (before) 0x%x\n", ulValue);            
    if( on == 1 )
        ulValue = 0x3; // compounding
    else 
        ulValue = 0x2; // linear
    printf("Compound register (after) 0x%x\n", ulValue);   
    err=dc1394_set_control_register( camera, 0x1A04, ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set HDR mode on\n"); 
        
}



// seth HDR
dc1394error_t setHDR( dc1394camera_t *camera, int on )
{
    // Create a variable to hold the register values
    dc1394error_t err;
    uint32_t ulValue = 0;

    // Set 0x0F (Pixel Operation Mode) to be the sensor register to be written to
    err=dc1394_set_control_register( camera, 0x1A00, 0x0F );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set control register to Pixel Operation Mode\n");
    
    // Flip bit 6 of the register to 1 to enable HDR mode
    err=dc1394_get_control_register( camera, 0x1A04, &ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Get HDR register\n");
    printf("HDR control register (before) 0x%x\n", ulValue);            
    if( on == 1 )
        ulValue = ulValue | 0x40;
    else
        ulValue = ulValue & 0xBF; 
    printf("HDR control register (after) 0x%x\n", ulValue);   
    err=dc1394_set_control_register( camera, 0x1A04, ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set HDR mode on\n");;   
        
    // Set 0x0A (Shutter Width Control) to be the sensor register to be written to
    err=dc1394_set_control_register( camera, 0x1A00, 0x0A ); //  
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set control register to Shutter Width Control\n");
    
    // Flip bit 8 of the register to 1 to enable automatic knee point timing
    // It seems this bit is on by default
    ulValue = 0;
    err=dc1394_get_control_register( camera, 0x1A04, &ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Get Shutter register\n");    
    printf("Shutter control register (before) 0x%x\n", ulValue); 
    if( on == 1 )
        ulValue = ulValue | 0x164; // auto knee point timing must be on always
    else
        ulValue = ulValue & 0x164; // auto knee point timing must be on always    
    printf("Shutter control register (after) 0x%x\n", ulValue);     
    err=dc1394_set_control_register( camera, 0x1A04, ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set to automatic knee point timing\n");
    /*
    // jason edits for shutter control and gamma control
    uint32_t shutter_write=0x82000017;
    uint32_t gamma_write  =0x82000001;
    err=dc1394_set_control_register( camera, 0x818, gamma_write );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"cannot Get gamma register\n");
    printf("gamma control register (Jason edits) 0x%x\n", ulValue);
    
    err=dc1394_set_control_register( camera, 0x81C, shutter_write );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"cannot Get shutter_j register\n");
    printf("shutter control register (Jason edits) 0x%x\n", ulValue);
    */
}



// seth auto exposure ang gain region
dc1394error_t setAutoRegion( dc1394camera_t *camera )
{
    // Create a variable to hold the register values
    dc1394error_t err;
    uint32_t ulValue = 0;
    
    // for each tile
    printf("Set auto region\n");
    for( int n = 0; n < 25; n++ )
    {

        // Set 0x0F (Pixel Operation Mode) to be the sensor register to be written to
        err=dc1394_set_control_register( camera, 0x1A00, 0x80 + n );
        printf("Set control register of tile 0x%x\n",0x80 + n);
        DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Set control register of tile\n");

        err=dc1394_set_control_register( camera, 0x1A04, 0xF4 );
//         // set the control register
//         if( n < 10 || 20 <= n )
//         {
//             // turn of these tiles
//             err=dc1394_set_control_register( camera, 0x1A04, 4 );
//         }
        
        // read the status of this register
        usleep(1e5);
        err=dc1394_get_control_register( camera, 0x1A04, &ulValue );
        printf("Tile %i value is 0x%x\n",n,ulValue);
        
    }    
}

dc1394error_t initializeCamera(dc1394camera_t *camera){
     dc1394error_t err;
     
     /*-----------------------------------------------------------------------
     *  reset camera
     *-----------------------------------------------------------------------*/
    err=dc1394_camera_reset(camera);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could reset camera");
    usleep(1e5);
     
    
    /*-----------------------------------------------------------------------
     *  setup capture
     *-----------------------------------------------------------------------*/
    err=dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_LEGACY ); // DC1394_OPERATION_MODE_1394B exploit 
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set 1394A mode"); 
    usleep(1e5);
    
    err=dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_400 );//DC1394_ISO_SPEED_800  exploit 
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set iso speed");
    usleep(1e5);
    
    // set the image mode
    err=dc1394_video_set_mode(camera, DC1394_VIDEO_MODE_640x480_MONO8 ); // MONO16 didnt work 
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set video mode\n");
    
    // set frame rate
    err=dc1394_video_set_framerate(camera,DC1394_FRAMERATE_15);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set frame rate right\n");
    usleep(1e5);
    
    // create 4 buffers
    err=dc1394_capture_setup(camera, 2, DC1394_CAPTURE_FLAGS_DEFAULT );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera\n");
    usleep(1e5);
    
    // set 12-to-10 bit compounding off (we do this directly using the camera regs)
    err=dc1394_feature_set_power(camera, DC1394_FEATURE_GAMMA, DC1394_OFF);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set HDR mode right\n");
    usleep(1e5);
/*
    err=dc1394_feature_set_power(camera, DC1394_FEATURE_SHUTTER, DC1394_ON);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set shutter mode right\n");
    usleep(1e5);

    // recent edits 18.05 22/8
    uint32_t val=100s;
    err=dc1394_feature_set_value(camera, DC1394_FEATURE_GAMMA,val );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not get gamma mode right\n");
    usleep(1e5);
    uint32_t valshutter=17;
    err=dc1394_feature_set_value(camera, DC1394_FEATURE_SHUTTER,valshutter );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set shutter mode right\n");
    usleep(1e5);
  */  
    setFrameFormat( camera );
    setCompounding( camera, hd ); 
    setHDR( camera, !comp );
    setLuminosity( camera, ecount );
    // setLuminosity( camera, 55 );
    // setAutoRegion( camera );
   
    /*-----------------------------------------------------------------------
     *  have the camera start sending us data
     *-----------------------------------------------------------------------*/
    err=dc1394_video_set_transmission(camera, DC1394_ON);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not start camera iso transmission\n");
    
}
// reading camera parameters using calibration
static bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

// reading detector parameters using calibration 
static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    //fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}
void readArucoFiles(int total_camera){
    camMatrix.resize(total_camera);
    distCoeffs.resize(total_camera);
    // reading all files for aruco//////////
   
    bool read = readDetectorParameters("detector_params.yml", detectorParams);
    if(!read) {
       	std::cerr << "Invalid detector parameters file" << std::endl;
        play =false;
    	 //return 0;
    }
    for (int i=0;i<total_camera;i++){
        char filename[50];
        sprintf(filename,"../new_attempt/calibration_files/camera_%d_checker.yml",i);
        bool readOk = readCameraParameters(filename, camMatrix[i], distCoeffs[i]);
        if(!readOk) {
            std::cerr << "Invalid camera file for camera number: "<<i+1 << std::endl;
            play =false;
            //return 0;
        }
    }

}
void getEulerAngles(cv::Mat &rotCamerMatrix,cv::Vec3d &eulerAngles){

    cv::Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    cv::decomposeProjectionMatrix( cv::Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}




void makeSense(cv::Vec3d tvec,cv::Vec3d rvec,int markerID){
    if(markerID<50){ // fixed markers
        cv::Mat rotMatrix;
		cv::Rodrigues(rvec,rotMatrix);
        transpose(rotMatrix,f_rotMat);
        f_tvec=-1*tvec;
        f_markerID=markerID;
        //
        
    }
    else{
        
        if (markerID>50){  // only moving markers
            cv::Matx33d rotMattoe0( 0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0);// if all x pointing in same direction
            cv::Vec3d reading=(rotMattoe0*(f_rotMat*(tvec+f_tvec)))+transtoe0[f_markerID];
            std::cerr << "using fixed marker ID:" << f_markerID << std::endl;
            std::cerr<<"origin to truck :" <<markerID<<"\t" << reading<< std::endl;
            
            cv::Mat rotationMatrix;
            cv::Rodrigues(rvec,rotationMatrix);
            cv::Vec3d angle_rot;
            getEulerAngles(rotationMatrix,angle_rot);
            std::cerr << "rotation angle(deg):" << angle_rot << std::endl;
            
            if (sent_data[markerID-51]==0){
                UDPfarewell(markerID,reading,angle_rot);// angles in degreee and x,y,z 
                sent_data[markerID-51]=1;
            }
            
            else{
                std::cout << "skipped:"<<markerID<<std::endl;
            }
        }
    }
}

 void arucoPipeline(cv::Mat img,int camera_number) {
    f_markerID=0; // reset for each frame
    std::vector<int> markerIds;
    std::vector< std::vector<cv::Point2f> > markerCorners,rejectedCandidates;
    std::vector< cv::Vec3d >  rvecs, tvecs;
    // fixed and moving markers have different dicitionary hence the search algorithm is changed 
    cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 
    cv::aruco::detectMarkers(img,dictionary,markerCorners,markerIds,detectorParams,rejectedCandidates);
    cv::aruco::drawDetectedMarkers(img,markerCorners,markerIds);
    cv::aruco::drawDetectedMarkers(img,rejectedCandidates , cv::noArray(), cv::Scalar(100, 0, 255)); // rejected candidates
    /*if (markerIds.size()>0){
        bool found_fixedM=0;
        for(unsigned int i = 0; i < markerIds.size(); i++){ 
                
                std::vector< std::vector<cv::Point2f> > single_markerCorner;
                std::vector<cv::Vec3d> single_rvec,single_tvec;
                single_markerCorner.resize(1);
                single_markerCorner[0]=markerCorners[i];
                //std::cout << "processed markercorner size"<< single_markerCorner.size()<<std::endl;
                
                
                if (markerIds[i]<50){
                        cv::aruco::estimatePoseSingleMarkers(single_markerCorner, markerLength_fixed, camMatrix[camera_number], distCoeffs[camera_number], single_rvec, single_tvec);
                        //std::cout<<"fixed markers:"<<markerIds[i]<<std::endl;
                        cv::aruco::drawAxis(img, camMatrix[camera_number], distCoeffs[camera_number], single_rvec[0], single_tvec[0],
        		                  markerLength_fixed*0.5f);
                        makeSense(single_tvec[0],single_rvec[0],markerIds[i]);
                        found_fixedM=1;
                }
        }
    std::vector<int> markerIds_m;
    std::vector< std::vector<cv::Point2f> > markerCorners_m,rejectedCandidates_m;
    std::vector< cv::Vec3d >  rvecs_m, tvecs_m;

    cv::Ptr<cv::aruco::Dictionary> dictionary_m=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250); // moving  
    cv::aruco::detectMarkers(img,dictionary_m,markerCorners_m,markerIds_m,detectorParams,rejectedCandidates);
    cv::aruco::drawDetectedMarkers(img,markerCorners_m,markerIds_m);
    cv::aruco::drawDetectedMarkers(img,rejectedCandidates , cv::noArray(), cv::Scalar(100, 0, 255)); // rejected candidates
    //sending moving markers
      if(found_fixedM==1){
        for(unsigned int i = 0; i < markerIds_m.size(); i++){
                std::vector< std::vector<cv::Point2f> > single_markerCorner;
                std::vector<cv::Vec3d> single_rvec,single_tvec;
                single_markerCorner.resize(1);
                single_markerCorner[0]=markerCorners_m[i];
                if(markerIds[i]>50){
                        cv::aruco::estimatePoseSingleMarkers( single_markerCorner, markerLength_moving, camMatrix[camera_number], distCoeffs[camera_number],  single_rvec,single_tvec);//verify
                        //std::cout<<"moving markers:"<<markerIds[i]<<std::endl;
                        //std::cout<<"moving markers:"<<single_rvec.size()<<std::endl;
                        cv::aruco::drawAxis(img, camMatrix[camera_number], distCoeffs[camera_number], single_rvec[0], single_tvec[0],
        		                  markerLength_moving*0.5f);
                        makeSense(single_tvec[0],single_rvec[0],markerIds[i]);
                }

        }
      }
    }*/
    
    // if markers have same dicitionnay 
    if (markerIds.size()>0) {
      //sending on fixed marker first
      bool found_fixedM=0;
      for(unsigned int i = 0; i < markerIds.size(); i++){ 
                
                std::vector< std::vector<cv::Point2f> > single_markerCorner;
                std::vector<cv::Vec3d> single_rvec,single_tvec;
                single_markerCorner.resize(1);
                single_markerCorner[0]=markerCorners[i];
                //std::cout << "processed markercorner size"<< single_markerCorner.size()<<std::endl;
                
                
                if (markerIds[i]<50){
                        cv::aruco::estimatePoseSingleMarkers(single_markerCorner, markerLength_fixed, camMatrix[camera_number], distCoeffs[camera_number], single_rvec, single_tvec);
                        //std::cout<<"fixed markers:"<<markerIds[i]<<std::endl;
                        cv::aruco::drawAxis(img, camMatrix[camera_number], distCoeffs[camera_number], single_rvec[0], single_tvec[0],
        		                  markerLength_fixed*0.5f);
                        makeSense(single_tvec[0],single_rvec[0],markerIds[i]);
                        found_fixedM=1;
                }
      }
      //sending moving markers
      if(found_fixedM==1){
      for(unsigned int i = 0; i < markerIds.size(); i++){
                std::vector< std::vector<cv::Point2f> > single_markerCorner;
                std::vector<cv::Vec3d> single_rvec,single_tvec;
                single_markerCorner.resize(1);
                single_markerCorner[0]=markerCorners[i];
                if(markerIds[i]>50){
                        cv::aruco::estimatePoseSingleMarkers( single_markerCorner, markerLength_moving, camMatrix[camera_number], distCoeffs[camera_number],  single_rvec,single_tvec);//verify
                        //std::cout<<"moving markers:"<<markerIds[i]<<std::endl;
                        //std::cout<<"moving markers:"<<single_rvec.size()<<std::endl;
                        cv::aruco::drawAxis(img, camMatrix[camera_number], distCoeffs[camera_number], single_rvec[0], single_tvec[0],
        		                  markerLength_moving*0.5f);
                        makeSense(single_tvec[0],single_rvec[0],markerIds[i]);
                }

      }
      }
                //rvecs[i]=single_rvec[0];
                //tvecs[i]=single_tvec[0];
                //makeSense(single_tvec[0],single_rvec[0],markerIds[i]);
    
    



    }
 }

dc1394error_t cameraCaptureSingle(dc1394camera_t* camera ,int camera_no){
    dc1394error_t err;
    dc1394video_frame_t* frame=NULL;
    dc1394video_frame_t frame_buffer;
    unsigned int stamp;
    unsigned char* pStamp = (unsigned char*) &stamp;
    cv::Mat BImage = cv::Mat( 480, 640, CV_8UC1 );
    cv::Mat dispImage = cv::Mat( 480, 640, CV_8UC3 );
    cv::Mat finalImage = cv::Mat( 480, 640, CV_8UC3 );
    
    /*-----------------------------------------------------------------------
    *  capture one frame
    *-----------------------------------------------------------------------*/
    err=dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not capture a frame\n");
    
    /*-----------------------------------------------------------------------
    *  check if frame is corrupt
    *-----------------------------------------------------------------------*/      
    if( dc1394_capture_is_frame_corrupt( camera, frame) )
        printf("\n[DEBUG] frame is corrupt!\n"); 
     
    memcpy( &frame_buffer, frame, sizeof(dc1394video_frame_t) );
     /* strictly little endian... */
    pStamp[ 0 ]  = frame_buffer.image[3];
    pStamp[ 1 ]  = frame_buffer.image[2];
    pStamp[ 2 ]  = frame_buffer.image[1];
    pStamp[ 3 ]  = frame_buffer.image[0];
    
    /*-----------------------------------------------------------------------
    *  copy pointer to opencv
    *-----------------------------------------------------------------------*/          
    BImage.data = frame_buffer.image;
    
    /*-----------------------------------------------------------------------
    *  convert to color image
    *-----------------------------------------------------------------------*/
    cvtColor( BImage, dispImage, CV_BayerRG2RGB );
    
    finalImage=dispImage.clone(); // So Deep copy 
    
     
    arucoPipeline(finalImage,camera_no);
    
    cv::Mat undistImg;
    cv::undistort(finalImage,undistImg,camMatrix[camera_no], distCoeffs[camera_no]);
    
    frames[camera_no]=undistImg;
    
    dc1394_capture_enqueue(camera,frame);
}
void keyPress(std::vector<dc1394camera_t*> cameras,dc1394camera_list_t * list ){
    int key;
    dc1394error_t err;
    key = cv::waitKey(10);
      
    /*-----------------------------------------------------------------------
    *  Handle keys
    *-----------------------------------------------------------------------*/
    if( key == 27 ) // esc
    {
        play = false;            
    }  
    else if( key == 104 ) // h
    { 
      // toggle HDR
      hd = !hd;
      printf("\nHDR is %i\n", hd );
      for(int ii=0;ii<(int)list->num;ii++){
        setHDR( cameras[ii], hd );
      }
      
    }
    else if(key == 's') {
        // jason edits for shutter control and gamma control
        uint32_t shutter_write=0x82000017;
        uint32_t gamma_write  =0x82000001;
        for(int ii=0;ii<(int)list->num;ii++){
            err=dc1394_set_control_register( cameras[ii], 0x818, gamma_write );
            //DC1394_ERR_CLN_RTN(err,cleanup_and_exit(cameras[ii]),"cannot Get gamma register\n");
            printf("gamma control register (Jason edits) is set \n");
    
            err=dc1394_set_control_register( cameras[ii], 0x81C, shutter_write );
            //DC1394_ERR_CLN_RTN(err,cleanup_and_exit(cameras[ii]),"cannot Get shutter_j register\n");
            printf("shutter control register (Jason edits) is set\n");
        }
    }
    else if( key == 99 ) // c
    { 
       // toggle HDR
       comp = !comp;
        printf("\nCompounding is %i\n", comp );
        for(int ii=0;ii<(int)list->num;ii++){
        setCompounding( cameras[ii], comp );
        }
        
    }
    else if( key == 82 ) // up arrow
    {
        ecount = ecount + 1e3;
        if( 65e3 < ecount )
            ecount = 65e3;
        printf("\nLuminosity is %i\n", ecount );
        for(int ii=0;ii<(int)list->num;ii++){
            setLuminosity( cameras[ii], ecount );
        }        
        
    }
    else if( key == 84  ) // down arrow
    {
        ecount = ecount - 1e3;
        if( ecount < 1e3 )
            ecount = 1e3;
        printf("\nLuminosity is %i\n", ecount ); 
        for(int ii=0;ii<(int)list->num;ii++){
            setLuminosity( cameras[ii], ecount );
        }        
        
    }
}

dc1394error_t cameraCleanup(dc1394camera_t* camera){
    dc1394error_t err;
    /*-----------------------------------------------------------------------
    *  stop data transmission
    *-----------------------------------------------------------------------*/
    err=dc1394_video_set_transmission(camera,DC1394_OFF);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not stop the camera?\n");

    
    
    /*-----------------------------------------------------------------------
    *  close camera
    *-----------------------------------------------------------------------*/
    dc1394_video_set_transmission(camera,DC1394_OFF);
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);
    
}

int main(int argc, char *argv[])
{

    /** Add cameras according to the USE_CAM defines above **/
	auto it = cameraID.end();

	#if USE_CAM_1 == 1
		it = cameraID.insert(it, 49712223537630471);
	#endif	
	#if USE_CAM_2 == 1
		it = cameraID.insert(it, 49712223537053658);
	#endif
	#if USE_CAM_3 == 1
		it = cameraID.insert(it, 49712223537053659);
	#endif
	#if USE_CAM_4 == 1
		it = cameraID.insert(it, 49712223537053660);
	#endif
	#if USE_CAM_5 == 1
		it = cameraID.insert(it, 49712223537630465);
	#endif
	#if USE_CAM_6 == 1
		it = cameraID.insert(it, 49712223537630451);
	#endif
	#if USE_CAM_7 == 1
		it = cameraID.insert(it, 49712223537630448);
	#endif
	#if USE_CAM_8 == 1
		it = cameraID.insert(it, 49712223537630468);
	#endif

	/** Optionally print which cameras are used **/

	print_cams(cameraID);

    UDPset();
    FILE* imagefile;
    std::vector<dc1394camera_t*> cameras;
    unsigned int width, height;
    //dc1394featureset_t features;
    dc1394_t* d;
    dc1394camera_list_t * list;
    dc1394error_t err;
   // qureying number of devices 
   d = dc1394_new ();
    if (!d)
        return 1;
    err=dc1394_camera_enumerate (d, &list);
    DC1394_ERR_RTN(err,"Failed to enumerate cameras");

    // no cameras in the list
    if (list->num == 0) {
        dc1394_log_error("No cameras found");
        return 1;
    }
    cameras.resize(list->num);
    frames.resize(list->num);
    for( int i =0;i<(int)list->num;i++){
        int j=0; //camera increment
        while (j<16){
            if ( cameraID.size()==(int)list->num){
                if (cameraID[i]==list->ids[j].guid){
                    break;
                }
                else {
                    j++;
                }
            }
            else{
                std::cout<<"Not enough GUID's to verify camera order"<<std::endl;
                return 1;
            }
        } 
		cameras[i] = dc1394_camera_new (d, list->ids[j].guid);
     		if (!cameras[i]) {
        		dc1394_log_error("Failed to initialize camera with guid %llx", list->ids[j].guid);
        		return 1;
     		}
		initializeCamera(cameras[i]);
        std::cout<< "Initialised "<<i+1<<" cameras"<< std::endl;
                
    }
    readArucoFiles(list->num);
    
    while(play){
            //sent_data[0]=0;sent_data[1]=0;
            sent_data.assign(sent_data.size(),0); // reinitialize sending data 
        
            for( int i =0;i<(int)list->num;i++){
                    
                    cameraCaptureSingle(cameras[i],i);
                    
                    
            }
            cv::Mat combImg=makeCombined(frames,700,2);
            cv::imshow("TruckLabImgs",combImg);
            keyPress(cameras,list);
    }

     // cleanup
    for( int i =0;i<(int)list->num;i++){
            cameraCleanup(cameras[i]);
    }
    dc1394_free (d);
    dc1394_camera_free_list (list);
    return 0;
}

