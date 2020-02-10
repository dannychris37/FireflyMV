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

// vector of camera IDs
vector<long int> cameraID;

Ptr<aruco::DetectorParameters> detectorParams= aruco::DetectorParameters::create();
// time measuring flags
#define MEAS_WHILE 		1
#define MEAS_WAIT 		1
#define MEAS_PROC		1
#define MEAS_SHOW		1

// time emasuring vars
timespec start_while, stop_while;
timespec start_wait, stop_wait;
timespec start_proc, stop_proc;
timespec start_show;

double delta_while, delta_wait, delta_proc, delta_show;

// printing
#define REALTIME_MONITORING		1
#define UPDATE_ITS				12
bool print_flag;

#endif
