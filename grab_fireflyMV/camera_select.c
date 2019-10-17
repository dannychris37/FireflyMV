#include "fireflymv.h"

using namespace std;

/** Defines for setting which cameras to use **/

#define USE_CAM_0 1
#define USE_CAM_1 1
#define USE_CAM_2 1
#define USE_CAM_3 1
#define USE_CAM_4 1
#define USE_CAM_5 1
#define USE_CAM_6 1
#define USE_CAM_7 1

/** Whether to show frame info for given camera **/

#define SHOW_FRAME_INFO 0
#define FRAME_INFO_CAM 0

/** Struct and methods for printing which cameras are used **/

struct cam_entry {
	char const *cam_name;
	long int cam_id;
};

struct cam_entry cams[] = {
	"CAM0", 49712223537630471,
	"CAM1", 49712223537053658,
	"CAM2", 49712223537053659,
	"CAM3", 49712223537053660,
	"CAM4", 49712223537630451,
	"CAM5", 49712223537630465,
	"CAM6", 49712223537630468,
	"CAM7", 49712223537630448,
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

void addUsedCams(){

	/** Add cameras according to the USE_CAM defines at the top **/

	auto it = cameraID.end();

	#if USE_CAM_7 == 1
		it = cameraID.insert(it, cams[7].cam_id);
	#endif	
	#if USE_CAM_6 == 1
		it = cameraID.insert(it, cams[6].cam_id);
	#endif
	#if USE_CAM_0 == 1
		it = cameraID.insert(it, cams[0].cam_id);
	#endif
	#if USE_CAM_4 == 1
		it = cameraID.insert(it, cams[4].cam_id);
	#endif
	#if USE_CAM_3 == 1
		it = cameraID.insert(it, cams[3].cam_id);
	#endif
	#if USE_CAM_2 == 1
		it = cameraID.insert(it, cams[2].cam_id);
	#endif
	#if USE_CAM_1 == 1
		it = cameraID.insert(it, cams[1].cam_id);
	#endif
	#if USE_CAM_5 == 1
		it = cameraID.insert(it, cams[5].cam_id);
	#endif

	/** Optionally print which cameras are used **/

	print_cams(cameraID);

}