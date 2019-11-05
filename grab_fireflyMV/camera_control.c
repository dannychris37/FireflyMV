#include <stdio.h>
#include <stdlib.h>
#include "fireflyMV.h"

#ifndef _WIN32
#include <unistd.h>
#endif

// direclty write to camera registers to enable HDR and 12-to-10 bit compounding
bool hd    = false; // more stable measurements
bool comp  = true;
int  ecount = 44e3;
bool play = true;

/**  Releases the cameras and exits **/

void cleanup_and_exit(dc1394camera_t *camera) {
    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);
    exit(1);
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

/** Set 12-to-10 bit compounding **/

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

/** set HDR **/

dc1394error_t setHDR( dc1394camera_t *camera, int on )
{
    // Create a variable to hold the register values
    dc1394error_t err;
    uint32_t ulValue = 0;

    // Set 0x0F (Pixel Operation Mode) to be the sensor register to be written to
    err = dc1394_set_control_register( camera, 0x1A00, 0x0F );
    DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera),"Set control register to Pixel Operation Mode\n");
    
    // Flip bit 6 of the register to 1 to enable HDR mode
    err = dc1394_get_control_register( camera, 0x1A04, &ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Get HDR register\n");
    printf("HDR control register (before) 0x%x\n", ulValue);            
    
    if( on == 1 )	ulValue = ulValue | 0x40;
    else 		    ulValue = ulValue & 0xBF; 
    
    printf("HDR control register (after) 0x%x\n", ulValue);   
    err = dc1394_set_control_register( camera, 0x1A04, ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera), "Set HDR mode on\n");;   
        
    // Set 0x0A (Shutter Width Control) to be the sensor register to be written to
    err = dc1394_set_control_register( camera, 0x1A00, 0x0A ); //  
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera), "Set control register to Shutter Width Control\n");
    
    // Flip bit 8 of the register to 1 to enable automatic knee point timing
    // It seems this bit is on by default
    ulValue = 0;
    err = dc1394_get_control_register( camera, 0x1A04, &ulValue );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera)," Get Shutter register\n");    
    printf("Shutter control register (before) 0x%x\n", ulValue); 
    
    if( on == 1 )	ulValue = ulValue | 0x164; // auto knee point timing must be on always
    else        	ulValue = ulValue & 0x164; // auto knee point timing must be on always    
    
    printf("Shutter control register (after) 0x%x\n", ulValue);     
    err = dc1394_set_control_register( camera, 0x1A04, ulValue );
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

/** Set auto exposure ang gain region **/

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
        DC1394_ERR_CLN_RTN(
        	err,
        	cleanup_and_exit(camera),
        	"Set control register of tile\n"
        );

        err = dc1394_set_control_register( 
        	camera, 
        	0x1A04, 
        	0xF4 
        );
//         // set the control register
//         if( n < 10 || 20 <= n )
//         {
//             // turn of these tiles
//             err=dc1394_set_control_register( camera, 0x1A04, 4 );
//         }
        
        // read the status of this register
        usleep(1e5);
        err = dc1394_get_control_register( 
        	camera, 
        	0x1A04, 
        	&ulValue 
        );
        printf("Tile %i value is 0x%x\n",n,ulValue);
        
    }    
}

dc1394error_t initializeCamera(dc1394camera_t *camera){
     
    dc1394error_t err;
     
    /** Reset camera **/
    err = dc1394_camera_reset(camera);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not reset camera");
    usleep(1e5);
    
    /** Setup capture **/

    // DC1394_OPERATION_MODE_1394B exploit
    err = dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_LEGACY );  
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set 1394A mode"); 
    usleep(1e5);
    
    // DC1394_ISO_SPEED_800  exploit
    err = dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_400);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set iso speed");
    usleep(1e5);
    
    // set the image mode
    err = dc1394_video_set_mode(camera, DC1394_VIDEO_MODE_640x480_MONO8 ); // MONO16 didnt work 
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set video mode\n");
    
    // set frame rate
    err = dc1394_video_set_framerate(camera,DC1394_FRAMERATE_15);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not set frame rate right\n");
    usleep(1e5);
    
    // create 4 buffers
    err = dc1394_capture_setup(camera, 2, DC1394_CAPTURE_FLAGS_DEFAULT );
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera\n");
    usleep(1e5);
    
    // set 12-to-10 bit compounding off (we do this directly using the camera regs)
    err = dc1394_feature_set_power(camera, DC1394_FEATURE_GAMMA, DC1394_OFF);
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
   
    /**  Camera start sending **/

    err = dc1394_video_set_transmission(camera, DC1394_ON);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not start camera iso transmission\n");
    
}

/** Read camera parameters using calibration **/

static bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;

}

/** Functions for reading detector parameters using calibration **/

static bool readDetectorParameters(
	std::string filename, 
	cv::Ptr<cv::aruco::DetectorParameters> &params
	) {

    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened()) return false;

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

//** Handle keys **//

void keyPress(std::vector<dc1394camera_t*> cameras,dc1394camera_list_t * list ){

    int key;
    dc1394error_t err;
    key = cv::waitKey(10);
    
    // esc
    if( key == 27 ){

        play = false;

    }

    // h
    else if( key == 104 ){
		
		// toggle HDR
		hd = !hd;
		printf("\nHDR is %i\n", hd );
		for(int ii=0;ii<(int)list->num;ii++){

			setHDR( cameras[ii], hd );

		}
    }

    else if(key == 's') {

    	// jason edits for shutter control and gamma control
        uint32_t shutter_write	= 0x82000017;
        uint32_t gamma_write  	= 0x82000001;

        for(int ii=0;ii<(int)list->num;ii++){
        	
        	err=dc1394_set_control_register( cameras[ii], 0x818, gamma_write );
            //DC1394_ERR_CLN_RTN(err,cleanup_and_exit(cameras[ii]),"cannot Get gamma register\n");
            printf("gamma control register (Jason edits) is set \n");
    
            err=dc1394_set_control_register( cameras[ii], 0x81C, shutter_write );
            //DC1394_ERR_CLN_RTN(err,cleanup_and_exit(cameras[ii]),"cannot Get shutter_j register\n");
            printf("shutter control register (Jason edits) is set\n");

        }
    }

    // c
    else if( key == 99 )
    { 
       	// toggle HDR
       	comp = !comp;
        printf("\nCompounding is %i\n", comp );
        for(int ii=0;ii<(int)list->num;ii++){
        setCompounding( cameras[ii], comp );
        }
        
    }

    // up arrow
    else if( key == 82 ){

        ecount = ecount + 1e3;
        if( 65e3 < ecount )
            ecount = 65e3;
        printf("\nLuminosity is %i\n", ecount );
        for(int ii=0;ii<(int)list->num;ii++){
            setLuminosity( cameras[ii], ecount );
        }        
        
    }

    // down arrow
    else if( key == 84 )
    {
        ecount = ecount - 1e3;

        if( ecount < 1e3 ) ecount = 1e3;

        printf("\nLuminosity is %i\n", ecount ); 

        for(int ii=0; ii<(int)list -> num; ii++){

            setLuminosity( cameras[ii], ecount );

        }        
    }

}

/** Camera Cleanup **/

dc1394error_t cameraCleanup(dc1394camera_t* camera){

    dc1394error_t err;

    // stop data transmission
    err=dc1394_video_set_transmission(camera,DC1394_OFF);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not stop the camera?\n");   
    
    // close camera
    dc1394_video_set_transmission(camera,DC1394_OFF);
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);
    
}