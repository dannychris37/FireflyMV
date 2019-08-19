INCS = $(shell pkg-config --cflags opencv)
LIBS = $(shell pkg-config --libs opencv) 
all: bumblebee  fireflymv fireflymvstereo fireflymvstereologger
	
bumblebee:
	g++ -o grab_Bumblebee_image grab_Bumblebee_image.c -ldc1394 -lopencv_core -lopencv_imgproc -lopencv_highgui 

fireflymv:
	g++ -std=c++11 -o grab_FireflyMV_image grab_FireflyMV_image.c -ldc1394 -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_aruco -lopencv_videoio -lopencv_calib3d
	
fireflymvstereo:
	g++ -o grab_FireflyMVStereo_image grab_FireflyMVStereo_image.c -ldc1394 -lopencv_core -lopencv_imgproc -lopencv_highgui 	

fireflymvstereologger:
	g++  -o grab_FireflyMVStereo_logger grab_FireflyMVStereo_logger.c -ldc1394 -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs

fireflymv_multiple:
	g++ -std=c++11 -o grab_FireflyMV_multiple grab_FireflyMV_multiple.c -ldc1394 -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_aruco -lopencv_videoio -lopencv_calib3d $(LIBS)

fireflymv_some:
	g++ -std=c++11 -o grab_FireflyMV_some grab_FireflyMV_some.c -ldc1394 -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_aruco -lopencv_videoio -lopencv_calib3d $(LIBS) 
