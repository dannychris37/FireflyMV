#include "fireflyMV.h"

// Computes the bearing in degrees from the point A(a1,a2) to
// the point B(b1,b2). Note that A and B are given in terms of
// screen coordinates.
double bearing(double a1, double a2, double b1, double b2) {
    static const double TWOPI = 6.2831853071795865;
    static const double RAD2DEG = 57.2957795130823209;
    // if (a1 = b1 and a2 = b2) throw an error 
    double theta = atan2(b1 - a1, a2 - b2);
    if (theta < 0.0)
        theta += TWOPI;
    return RAD2DEG * theta;
}

void computeAverage(int i){

    for(int k = 0; k < 3; k++){
        avgCoords[k] = 0;
        avgAngles[k] = 0;
    }
    cameraCount = 0;

    for(int j = 0; j < 8; j++){

        if(dataToSend[i][j].valuesStored){

            if(print){
                std::cout << "\nAVG: Marker " << i << " found by camera "<< j << std::endl;
                std::cout << "AVG: Using fixed marker ID: " << dataToSend[i][j].fixedMarker << std::endl;
                std::cout << "AVG: Coordinates:\t" << dataToSend[i][j].coords << std::endl;
                std::cout << "AVG: Angle:\t\t" << dataToSend[i][j].angles << std::endl << std::endl;
            }

            // For averaging
            for(int k = 0; k < 3; k++){
                avgCoords[k] += dataToSend[i][j].coords[k];
                avgAngles[k] += dataToSend[i][j].angles[k];
            }
            cameraCount++;

            // For diffs
            for(int di = j+1; di < 8; di++){
                if(dataToSend[i][di].valuesStored){
                    diffs[j][di][0] = abs(dataToSend[i][di].coords[0] - dataToSend[i][j].coords[0]) * 1000;
                    diffs[di][j][0] = diffs[j][di][0];
                    diffs[j][di][1] = abs(dataToSend[i][di].coords[1] - dataToSend[i][j].coords[1]) * 1000;
                    diffs[di][j][1] = diffs[j][di][1];
                }
            }
        }

    } // for j

    for(int k = 0; k < 3; k++){
        avgCoords[k] /= cameraCount;
        avgAngles[k] /= cameraCount;
    }
}

void detectFirstMotion(int i){

    if(firstCnt < INIT_AVG_POINTS){ 

        //gather points for initial averaging
        compCoords += avgCoords;
        compAngles += avgAngles;
        firstCnt++;

    } else if(firstPos){ 

        //when points gathered compute first average
        prevCoords = compCoords/INIT_AVG_POINTS;
        prevAngles = compAngles/INIT_AVG_POINTS;
        compCoords = 0;
        compAngles = 0;
        firstCnt = 0;
        firstPos = false;
        prevState = cameraCount;

        if (print) {
            cout << "\nFMD: Assigning first coordinates to:\t" <<prevCoords<<endl;
            cout << "FMD: Sending first coordinates for marker " << i << endl;
            cout << "FMD: Coordinates to send:\t" << prevCoords << endl;
            cout << "FMD: Angles to send:\t\t" << prevAngles << endl;
            cout << "FMD: Camera count and state are "<< prevState << endl;
        }
        UDPfarewell(i, prevCoords, prevAngles);

    } else{     

        // if first average already computed then check if second average is at a minimal distance
        // in which case the truck has moved away from initial location
        compCoords /= INIT_AVG_POINTS;
        compAngles /= INIT_AVG_POINTS;
        double distance = sqrt(
            pow(prevCoords[0] - compCoords[0], 2) +
            pow(prevCoords[1] - compCoords[1], 2)
        );

        if(distance >= MIN_DIST){

            currentBearing = bearing(prevCoords[0], prevCoords[1], compCoords[0], compCoords[1]);

            prevCoords = compCoords;
            prevAngles = compAngles;
            transStartCoords = prevCoords; // to be used for transition later

            detectingMotion = false;
            if (print){
                cout<<"\nFMD: Motion detected at point:\t"<<prevCoords<<" Distance: "<<distance<<endl;
                cout<<"FMD: Angle set to: "<<currentBearing<<endl;
            }
            UDPfarewell(i, prevCoords, prevAngles);

        } else {

            if (print) cout<<"\nFMD: Motion not yet detected at point:\t"<<compCoords<<" Distance: "<<distance<<endl;
            compCoords = 0;
            compAngles = 0;
            firstCnt = 0;

        }
    }

}

void checkState(){

    //if(print) cout<<"\nSTATE: Camera count - "<<cameraCount<<"; previous state - "<<prevState<<endl;

    if(cameraCount != prevState){

        if(print) cout<<"\nSTATE: Changed state to "<<cameraCount<<" cameras"<<endl;
        if(inTransition){
            if(print) cout<<"\nSTATE: Transition break. Changing start coordinates from "<<transStartCoords<<" to "<<prevCoords<<endl;
            transStartCoords = prevCoords;
        }
        prevState = cameraCount;
        stepCount = 1;
        inTransition = true;
        // first keep close to start
        compCoords = (transStartCoords * (TRANS_STEPS - stepCount) + avgCoords * stepCount) / TRANS_STEPS;

    } else if (inTransition){

        if(stepCount == TRANS_STEPS){ // transition done

            transStartCoords = avgCoords;
            inTransition = false;

        } else { // in transition

            // transition from start to average bit by bit
            stepCount++;
            if(print) 
                cout<<"\nSTATE: Transition step "<<stepCount<<endl;
            compCoords = (transStartCoords * (TRANS_STEPS - stepCount) + avgCoords * stepCount) / TRANS_STEPS;

        }

    } else { // no state change
        compCoords = avgCoords;
    }

}

void checkAngleAndDist(int i){
    // mitigating large angle shifts
    compBearing = bearing(prevCoords[0], prevCoords[1], compCoords[0], compCoords[1]);
    float angleDiff = abs(compBearing - currentBearing);
    if(angleDiff > 180) angleDiff = 360 - angleDiff;
    if(angleDiff < MAX_DEGREES){

        double distance = sqrt(
            pow(prevCoords[0] - compCoords[0], 2) +
            pow(prevCoords[1] - compCoords[1], 2)
        );

        if(print) cout<<"\nANGLE: Angle diff is below/equal set value |"<<compBearing<<" - "<<currentBearing<<"| = "<<angleDiff<<endl;

        if(distance <= MAX_DIST){

            if(print) cout<<"\nDIST: Distance is below/equal set value: "<<distance<<endl;
            
            currentBearing = compBearing;

            // store in case transition is needed next iteration
            if(!inTransition) transStartCoords = compCoords;

            // no transition needed
            prevCoords = compCoords;

        }



    } else{ //else do nothing, send same coords

        if(print){
            cout<<"\nANGLE: Angle diff with last value is too large |"<<compBearing<<" - "<<currentBearing<<"| = "<<angleDiff<<endl;
            cout<<"ANGLE: Previously sent:\t "<<prevCoords<<endl;
            cout<<"ANGLE: Current coords:\t\t "<<compCoords<<endl;
        }
    }

    if(print) {
        cout << fixed;
        cout << setprecision(2); 
        cout << "\nDiffs for marker " << i << " (in cm): " << endl;
        for(int j=0; j<8;j++){
            if(j==0)
                cout<< "\t  CAM"<<j<<"  ";
            else
                cout<< "\t    CAM"<<j<<"  ";
        }
        cout<<endl;
        for(int j=0; j<8;j++){
            cout<< "CAM"<<j<<"| ";
            for(int di=0; di<8;di++){
                cout<<"["<<diffs[j][di][0]<<","<<diffs[j][di][1]<<"]\t";
            }
            cout<<"|"<<endl;
        }
        cout << fixed;
        cout << setprecision(6);
        cout << "\nSEND: Sending data for marker " << i << endl;
        cout << "SEND: Coordinates to send:\t" << prevCoords << endl;
        cout << "SEND: Angles to send:\t\t" << avgAngles << endl;
    }
    UDPfarewell(i, prevCoords, avgAngles);
}