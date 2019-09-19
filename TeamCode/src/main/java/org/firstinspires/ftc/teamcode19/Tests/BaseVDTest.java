package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous(group = "Test")
public class BaseVDTest extends BaseAuto {
    private double landerAwayTurningAmount = 56;
    private double threshold = 3;
    private double dumpMarkerX = -53;
    private double landerAndWallAllignY = -1;
    private double[] AllWOrientation = {-95,-175,95,5}; // CraterBaseCorridor 0,1,2,3
    private double[] AllCOrientation = {-53,50,131,-140}; // BaseLanderCorridor 0,1,2,3
    public void loop(){
        int VUMarkResult2 = waitUntilVumarkRecognized();

        if(VUMarkResult2 != -1){
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            flash();
            telemetry.addData("", currOrientation[0]);
            telemetry.addData("", currOrientation[1]);
            telemetry.addData("Using VUMark", currHeading);
            telemetry.addData("turning target: ",AllWOrientation[VUMarkResult2]);
            telemetry.addData("turning angle: ", turningAmount);
            telemetry.update();
            turningAmount = headingOffset(currHeading, AllWOrientation[VUMarkResult2]);
            if(Math.abs(turningAmount+150.0) > 15) turningAmount = -150;
        }
        else{
            turningAmount = -150;
            telemetry.addData("using hardcode", turningAmount);
            telemetry.update();
        }

        turn(turningAmount, 0.15, 1);
        //Todo: change to VUMARK DISTANCE
        currPosition = getPositionFromVumark();
        XOffset = dumpMarkerX - currPosition[0];
        YOffset = landerAndWallAllignY - currPosition[1];
        telemetry.addData("curX: ", currPosition[0]);
        telemetry.addData("curY: ", currPosition[1]);
        telemetry.addData("XOffset: ", XOffset);
        telemetry.addData("YOffset: ", YOffset);
        moveInchesHighSpeedEncoder(5.3, 0, 0.1, 1,1,0.15,0.35,0.1);
        moveInchesHighSpeedEncoder(0, -47, 0.6, 4,8,0.1,0.55,0.15);
        wait(15000);
        requestOpModeStop();
    }
}
