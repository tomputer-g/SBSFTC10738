package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode19.BaseAuto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
@Disabled
@Autonomous
public class WallVUMarkTesting extends BaseAuto {
    private double x = 70, y = -34, theta = 0;
    @Override
    public void init() {
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 30000;
        msStuckDetectInitLoop = 10000;
        super.init();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop(){
        boolean targetVisible = false;
        wait(200);
        light.setPosition(0.5);
        while(!targetVisible){
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                theta = rotation.thirdAngle;
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.update();
                turningAmount = headingOffset(theta, 3);
                turn(turningAmount,0.15,1);
               //  express position (translation) of robot in inches.
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
                VectorF translation = lastLocation.getTranslation();
                x = translation.get(0)/mmPerInch;
                y = translation.get(1)/mmPerInch;
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                telemetry.addData("x", x);
                telemetry.addData("y",y);
                // express the rotation of the robot in degrees.


                turn(180,0.15,3);
                moveInchesHighSpeedEncoder(68-x,0,0.2,3,3,0.15,0.3,0.1);
                moveInchesHighSpeedEncoder(0,-(34+y),0.2,3,3,0.15,0.3,0.1);
            }
            else {
                telemetry.addData("Visible Target", "none");
                telemetry.update();
            }
        }



        requestOpModeStop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
