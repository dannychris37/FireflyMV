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

// vector of camera IDs
std::vector<long int> cameraID;

std::vector<dc1394camera_t*> cameras;

// context in which the library can be used
dc1394_t* d;

// list of cameras to be located by dc1394_camera_enumerate
dc1394camera_list_t * list;

// enum with error macros to be used for error reporting
dc1394error_t err;

cv::Ptr<cv::aruco::DetectorParameters> detectorParams= cv::aruco::DetectorParameters::create();
// time measuring flags
#define MEAS_WHILE 		1
#define MEAS_WAIT 		1
#define MEAS_PROC		1
#define MEAS_SHOW		1
#define NICE_PRINT		1

// time emasuring vars
timespec start_while, stop_while;
timespec start_wait[8], stop_wait[8];
timespec start_proc[8], stop_proc[8];
timespec start_show;

double delta_while, delta_wait[8], delta_proc[8], delta_show;

// thread mutex for printing
std::mutex mtx_wait, mtx_pose, mtx_pose_cnt, mtx_proc;	
std::condition_variable cnd_var_wait, cnd_var_pose, cnd_var_proc; // condition variable for print critical sections
int print_wait_cnt = 0, print_proc_cnt = 0, proc_cnt = 0;
bool can_print_wait_times = false, can_print_poses = false, can_print_proc_times = false;


#endif
