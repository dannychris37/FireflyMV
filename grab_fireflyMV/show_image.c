#include "fireflymv.h"

/** Show processed camera feed **/

cv::Mat makeCombined(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows) {

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