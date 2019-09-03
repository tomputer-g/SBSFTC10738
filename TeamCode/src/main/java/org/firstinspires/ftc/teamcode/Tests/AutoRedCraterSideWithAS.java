package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseAuto;

import java.sql.Time;

//@Autonomous(name = "Auto_Crater_Red_With_AS", group = "Final")
@Deprecated
public class AutoRedCraterSideWithAS extends BaseAuto {

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

        if(firstVumarkUsed)
            light.setPosition(1);
        moveInchesHighSpeedEncoder(-3.5, 0, 0.1,1,1,0.15,0.2,0.1);
        light.setPosition(0.5);

        redCraterMineral(TFOD_result);


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

        switch(TFOD_result){
            case -1:
                throw new IllegalArgumentException("TFOD returned -1");
            case 0:
                if(secondVumarkUsed)
                    light.setPosition(1);
                moveInchesHighSpeedEncoder(-5.5, 0, 0.15, 1,1,0.15,0.35,0.15);
                light.setPosition(0.5);
                moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.1);
                break;
            case 1:
                if(secondVumarkUsed)
                    light.setPosition(1);
                moveInchesHighSpeedEncoder(-4, 0, 0.15, 1,1,0.15,0.35,0.15);
                light.setPosition(0.5);
                moveInchesHighSpeedEncoder(0, -30.5, 0.4, 4,8,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(21,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(-21,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0, -6, 0.4, 4,8,0.1,0.55,0.1);
                break;
            case 2:
                if(secondVumarkUsed)
                    light.setPosition(1);
                moveInchesHighSpeedEncoder(-4, 0, 0.15, 1,1,0.15,0.35,0.15);
                light.setPosition(0.5);
                moveInchesHighSpeedEncoder(0, -18.5, 0.4, 4,8,0.1,0.55,0.15);
                moveInchesHighSpeedEncoder(13,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(-13,0,0.2,2,4,0.1,0.3,0.1);
                moveInchesHighSpeedEncoder(0, -17, 0.4, 4,8,0.1,0.55,0.1);
                break;
            default:
                if(secondVumarkUsed)
                    light.setPosition(1);
                moveInchesHighSpeedEncoder(-4, 0, 0.15, 1,1,0.15,0.35,0.15);
                light.setPosition(0.5);
                moveInchesHighSpeedEncoder(0, -35.5, 0.4, 4,8,0.1,0.55,0.1);
                break;
        }

        dropMarker();

        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_shoulder.setTargetPosition(-5450);
        grabber_shoulder.setPower(1);
        
        moveInchesHighSpeedEncoder(0,56.5, 1, 15,30,0.1,0.7,0.2);

        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        requestOpModeStop();
    }
}
