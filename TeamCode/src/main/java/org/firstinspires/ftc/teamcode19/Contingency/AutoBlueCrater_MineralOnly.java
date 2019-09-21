package org.firstinspires.ftc.teamcode19.Contingency;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous(group = "Plan B")
public class AutoBlueCrater_MineralOnly extends BaseAuto {

    @Override
    public void init() {
        super.init();
        getParams(false);
    }

    @Override
    public void loop() {
        stopTFOD();
        lowerRobot();
        turn(-landerAwayTurningAmount,0.2,threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3,3,3,0.1,0.1,0.1);

        if (waitUntilVumarkRecognized() == -1)
        {
            turningAmount = 53;
        }
        else
        {

            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndCraterOrientation);
            if (Math.abs(turningAmount-51) >=15)
                turningAmount = 51;
            else
                firstVumarkUsed = true;
        }
        turn(turningAmount, 0.15, 1);

        moveInchesHighSpeedEncoder(-3.5, 0, 0.1,1,1,0.15,0.2,0.1);
        switch(TFOD_result){
            case 0:
                moveInchesHighSpeedEncoder(0,-15.7,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-9.5,0,0.2,2,4,0.1,0.3,0.1);
                break;
            case 1:
                moveInchesHighSpeedEncoder(-10,0,0.3,3,6,0.1,0.55,0.15);
                break;
            case 2:
                moveInchesHighSpeedEncoder(0,15,0.3,3,6,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(-11,0,0.2,2,4,0.1,0.3,0.1);
                break;
        }
        requestOpModeStop();
    }
}
