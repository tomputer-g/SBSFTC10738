package org.firstinspires.ftc.teamcode19.Final;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.BaseAuto;

@Autonomous(group = "Final")
public class AutoBlueCraterMO_MF extends BaseAuto {
    private double landerAwayTurningAmount = 58;
    // The first turnning Angle of Auto, basically get off the lander
    private double landerAndCraterOrientation = 44;
    // Make the Robot parallel to crater
    private double grabberAndCraterOrientation = -124.75;
    // Used to align robot to the crater before moving to the final position

    public void loop(){
        stopTFOD();
        lowerRobot();

        // Clear the lander
        turn(-landerAwayTurningAmount,0.2,threshold);
        moveInchesHighSpeedEncoder(-8, -8, 0.3,3,3,0.1,
                0.1,0.1);

        // Attempt to see Vumark in order to align robot with the crater
        // Wait exists here:
        if (waitUntilVumarkRecognized() == -1) {
            Log.i("Vumark_Visibility", "False");
            turningAmount = 55.5;
            Log.i("LanderAndCraterAlign: ", "Overwrite: True");
        }
        else
        {
            Log.i("Vumark_Visibility", "True");
            refreshVumarkReading();
            currOrientation = getOrientationFromVumark();
            currHeading = currOrientation[2];
            turningAmount = headingOffset(currHeading, landerAndCraterOrientation);
            if (Math.abs(turningAmount-51) >=15) {
                turningAmount = 55.5;
                Log.i("LanderAndCraterAlign: ", "Overwrite: True");
            }
            else {
                Log.i("LanderAndCraterAlign: ", "Overwrite: False");
                Log.i("LanderAndCraterAlign: ", "" + turningAmount);
            }
        }
        Log.i("LanderAndCraterAlign: ", "" + turningAmount);
        turn(turningAmount, 0.15, 1);

        moveInchesHighSpeedEncoder(-3.5, 0, 0.1,1,1,0.15,
                0.2,0.1);


        // Move to position in order to push mineral. Push it. Come Back. Move to the wall
        switch(TFOD_result) {
            case 0:
                moveInchesHighSpeedEncoder(0, -15.7, 0.3, 3, 6, 0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(-10, 0, 0.2, 2, 4, 0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(11, 0, 0.2, 2, 4, 0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(0,13.2,0.2,3,6,0.1,0.55,0.15);
                break;
            case 1:
                moveInchesHighSpeedEncoder(-11, 0, 0.3, 3, 6, 0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(10, 0, 0.2, 2, 4, 0.1, 0.3, 0.1);
                break;
            case 2:
            default:
                moveInchesHighSpeedEncoder(0, 13, 0.3, 3, 6, 0.1, 0.55, 0.15);
                moveInchesHighSpeedEncoder(-10, 0, 0.2, 2, 4, 0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(11, 0, 0.2, 2, 4, 0.1, 0.3, 0.1);
                moveInchesHighSpeedEncoder(0,-15.5,0.2,3,6,0.1,0.55,0.15);
        }


        Log.i("turningAmount: ", ""+turningAmount);
        Log.i("currHeading: ", ""+currHeading);
        Log.i("g&cOrientation: ", ""+grabberAndCraterOrientation);

        turn(90, 0.15, 1);
        // Move Arm and slide to grabbing position
        grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_shoulder.setTargetPosition(grabberShoulderFinalPos);
        grabber_slide.setTargetPosition(grabberSlideFinalPos);
        grabber_shoulder.setPower(1);
        grabber_slide.setPower(1);

        // Change grabber shoulder motor mode. Get ready for Teleop.
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveInchesHighSpeedEncoder(0,3,0.2,3,6,0.1,0.55,0.15);

        wait(3000);

        requestOpModeStop();
    }
}
