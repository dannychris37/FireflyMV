#ifndef HEADER_SEEN
#define HEADER_SEEN

#define BILLION  1000000000L;
#define MILLION  1000000L;

#include <stdint.h>
#include <string.h>
#include <dc1394/dc1394.h>
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
#include <cstdio>
#include <cmath>

//thread libs
#include <thread>
#include <mutex>         
#include <condition_variable>

/** UDP libs **/

#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <ctime>

/** OpenCV libs **/

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/videoio/videoio.hpp"

using namespace std;
using namespace cv;

/** FRAME CAPTURING AND PROCESSING **/

// vector of all processed aruco frames
vector<Mat> frames;

// camera matrix and distance coefficients used in 
// readArucoFiles function
vector<Mat> camMatrix, distCoeffs;

// fixed marker rvec->rotmat,tvec (global declaration)
Matx33d f_rotMat[8];
Vec3d f_tvec[8];
int f_markerID[8];

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

// only for fixed markers
const float markerLength_fixed      = 0.0203;

// only for moving markers
const float markerLength_moving     = 0.013;//0.0097;

// structs used to store coordinates and angles of moving markers
typedef struct  {
	bool valuesStored;
	int fixedMarker;
	Vec3d coords;
	Vec3d angles;
} markerData;

markerData dataToSend[100][8];
bool markerFound[100] = {false};

/** SENSOR FUSION **/

// first motion detect
#define INIT_AVG_POINTS		10
#define MIN_DIST			0.01
int firstCnt = 0;
Vec3d prevCoords;
Vec3d prevAngles;
Vec3d compCoords;
Vec3d compAngles;
bool firstPos = true;
bool detectingMotion = true;

// vars for average and diff
Vec3d avgCoords;
Vec3d avgAngles;
int cameraCount;
Vec2d diffs[8][8] = {0,0};

// transition
#define TRANS_STEPS		25
int prevState;
int stepCount;
Vec3d sentCoords;
Vec3d transStartCoords;
bool inTransition = false;

//offset mitigation
#define MAX_DEGREES		120  // in degress
#define MAX_DIST      60   // in cm
double compBearing;
double currentBearing;

/** CAMERA DATA STRUCTURES **/

// vector of camera IDs
vector<long int> cameraID;

vector<dc1394camera_t*> cameras;

// context in which the library can be used
dc1394_t* d;

// list of cameras to be located by dc1394_camera_enumerate
dc1394camera_list_t * list;

// enum with error macros to be used for error reporting
dc1394error_t err;

/** TIME MEASUREMENT **/

Ptr<aruco::DetectorParameters> detectorParams= aruco::DetectorParameters::create();
// time measuring flags
#define MEAS_WHILE 		1
#define MEAS_WAIT 		1
#define MEAS_PROC		1
#define MEAS_SHOW		1
#define DIFF_OUTP   0

// time measuring vars
timespec start_while, stop_while;
timespec start_wait[8], stop_wait[8];
timespec start_proc[8], stop_proc[8];
timespec start_show;
timespec stop_wait7;

double delta_while, delta_wait[8], delta_proc[8], delta_show, delta_wait7;

/** THREADS AND PRINTING **/

// thread mutex for printing
mutex mtx_wait, mtx_pose_print, mtx_pose_cnt, mtx_proc;	
condition_variable cnd_var_wait, cnd_var_pose_print, cnd_var_pose_cnt, cnd_var_proc; // condition variable for print critical sections
int print_wait_cnt = 0, print_proc_cnt = 0, pose_cnt = 0;
bool can_print_wait_times = false, can_print_poses = false, can_print_proc_times = false;

//clear screen counter and printing flag
#define REALTIME_MONITORING		1
#define UPDATE_ITS				10
int cnt = 0;
bool print_flag = false;

#endif
