#ifndef HEADER_SEEN
#define HEADER_SEEN

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

cv::Ptr<cv::aruco::DetectorParameters> detectorParams= cv::aruco::DetectorParameters::create();

#endif