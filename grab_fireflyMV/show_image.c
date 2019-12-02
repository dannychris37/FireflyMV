#include "fireflyMV.h"
#include "camera_control.c"

/** Show processed camera feed **/

Mat makeCombined(vector<Mat>& vecMat, int windowHeight, int nRows) {

    // no of frames
    int N = vecMat.size();

    // if rows exceed no of frames
    nRows  = nRows > N ? N : nRows;

    // thickness of black edge ?
    int edgeThickness = 10;

    // make rows even
    int imagesPerRow = ceil(double(N) / nRows);


    int resizeHeight = floor(((floor(double(windowHeight - edgeThickness) / nRows)) ) - edgeThickness);


    int maxRowLength = 0;

    vector<int> resizeWidth;

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
    Mat combinedImage(windowHeight, windowWidth, CV_8UC3, Scalar(0, 0, 0));
    
    for (int k = 0, i = 0; i < nRows; i++) {

        int y = i * resizeHeight + (i + 1) * edgeThickness;
        int x_end = edgeThickness;

        for (int j = 0; j < imagesPerRow && k < N; k++, j++) {

                int x = x_end;

                Rect roi(x, y, resizeWidth[k], resizeHeight);
                Size s = combinedImage(roi).size();
                
                // change the number of channels to three
                Mat target_ROI(s,CV_8UC3);
                vecMat[k].copyTo(target_ROI);
                resize(target_ROI, target_ROI, s);
                
                if (target_ROI.type() != combinedImage.type()) {

                    target_ROI.convertTo(target_ROI, combinedImage.type());

                }

                target_ROI.copyTo(combinedImage(roi));
                x_end += resizeWidth[k] + edgeThickness;

        }
    }

    return combinedImage;

}

void showFrames(){

    if(MEAS_SHOW){

        clock_gettime(CLOCK_MONOTONIC, &start_show);

    }

    // show captured frames
    Mat combImg = makeCombined(frames, 800, 2);
    imshow("TruckLabImgs", combImg);
    keyPress(cameras, list);

    if(MEAS_WHILE || MEAS_SHOW){

        clock_gettime(CLOCK_MONOTONIC, &stop_while);

    }

    if(MEAS_SHOW){

        delta_show = ( stop_while.tv_sec - start_show.tv_sec )
             + (double)( stop_while.tv_nsec - start_show.tv_nsec )
               / (double)MILLION;

        if(print_flag)
            cout << "Frame show time: " << delta_show << "\n";

    }


    if(MEAS_WHILE){

        delta_while = ( stop_while.tv_sec - start_while.tv_sec )
             + (double)( stop_while.tv_nsec - start_while.tv_nsec )
               / (double)MILLION;

        if(print_flag){
            cout << "While loop time: " << delta_while << "\n";
            cout << "---------------------------------------------------\n";
        }
        

    }

    if(REALTIME_MONITORING){ // print every UPDATE_ITS iterations with non-rolling output
        
        cnt++;

        if(cnt == UPDATE_ITS) {

            system("clear");
            print_flag = true;
            cnt = 0;

        } else {

            print_flag = false;

        }

    }       
}