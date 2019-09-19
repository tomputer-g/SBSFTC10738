package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous(name = "AutoVumarkKnock", group = "Tests")
@Disabled
public class AutoVumarkCubeKnocking extends BaseAuto {

//    double[] targetPosOne = {-46.5,-16.8,0};
//    double[] targetOriOne  = {0,0,180}; //Right No.3

    double[] targetPosOne = {-36,-29,0};
    double[] targetOriOne  = {0,0,180}; //Right No.2

//    double[] targetPosOne = {0,0,0};
//    double[] targetOriOne  = {0,0,180}; //Right No.1


    @Override
    public void init(){
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 30000;
        initDrivetrain();
        initGrabber();
        initLifter();
        initVumark();
        initIMU();
    }

    @Override
    public void loop() {

        turn(-60, 0.3, THRESHOLD_90);
        moveInches(-16, 0, 0.3);

        setMode_RUN_WITH_ENCODER();
        setAllDrivePower(-0.1,0,0,0.1);
        waitUntilVumarkRecognized();
        setAllDrivePower(0);

        double[] currOri = getOrientationFromVumark();
        double headingOffset = headingOffset(currOri[2], targetOriOne[2]);
        turn(headingOffset,0.3,THRESHOLD_90);

        double[] currPos = getPositionFromVumark();
        double[] posOffset = posOffset(currPos, targetPosOne);
        moveInches(posOffset[0],posOffset[1],0.3);

        telemetry.addData("XOffset", posOffset[0]);
        telemetry.addData("YOffset", posOffset[1]);
        telemetry.addData("HeadingOffset", headingOffset);
        telemetry.update();

        requestOpModeStop();
    }



}
