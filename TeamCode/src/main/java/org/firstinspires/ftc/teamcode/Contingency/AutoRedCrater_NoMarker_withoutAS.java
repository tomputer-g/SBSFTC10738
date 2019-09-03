package org.firstinspires.ftc.teamcode.Contingency;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseAuto;

@Autonomous(group = "Plan B")
public class AutoRedCrater_NoMarker_withoutAS extends BaseAuto {

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



        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_shoulder.setTargetPosition(-5450);
        grabber_shoulder.setPower(1);

        moveInchesHighSpeedEncoder(0,30, 1, 15,30,0.1,0.7,0.2);


        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        requestOpModeStop();
    }
}
