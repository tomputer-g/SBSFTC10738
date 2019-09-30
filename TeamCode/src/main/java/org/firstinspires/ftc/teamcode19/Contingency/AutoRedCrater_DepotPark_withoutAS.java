package org.firstinspires.ftc.teamcode19.Contingency;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous(group = "Plan B")
@Disabled
public class AutoRedCrater_DepotPark_withoutAS extends BaseAuto {

    @Override
    public void init() {
        super.init();
        getParams(true);
    }

    @Override
    public void loop() {
        stopTFOD();
        lowerRobot(); //Lower+MoveArm

        turn(-landerAwayTurningAmount,0.2,threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3,3,3,0.1,0.1,0.1);

        if (waitUntilVumarkRecognized() == -1)
            turningAmount = 51;
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

        redCraterMineral(TFOD_result);

        if (firstVumarkUsed)
            flash();

        if (vumarkIsVisible())
        {
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndWallOrientation);
            if (Math.abs(turningAmount-33.8) >=15)
                turningAmount = 33.8;
            else
                secondVumarkUsed = true;
        }
        else
            turningAmount = 33.8;

        turn(turningAmount, 0.15, 1.5);


        moveInchesHighSpeedEncoder(-5.5, 0, 0.15, 1,1,0.15,0.35,0.15);
        moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.1);

        dropMarker();
        requestOpModeStop();
    }
}
