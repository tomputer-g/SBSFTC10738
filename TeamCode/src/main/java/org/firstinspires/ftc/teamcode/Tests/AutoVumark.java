package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseAuto;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.BaseOpMode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(group = "Test")
public class AutoVumark extends BaseAuto {

    @Override
    public void init(){
        msStuckDetectInit = 1000000000;
        msStuckDetectLoop = 30000;
        initDrivetrain();
        initGrabber();
                    initLifter();
                    initVumark();
                    initIMU();
                }

                @Override
                public void loop() {
                turn(90, 0.3, THRESHOLD_90);
                targetVisible = false;
                while (!targetVisible)
                {
                    for (VuforiaTrackable trackable : allTrackables)
                    {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                        {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null)
                    {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            // Provide feedback as to where the robot is located (if we know).
            moveInches(-3.5, 0, 0.3);
            telemetry.addData("Visible Target", "none");
        }

        double[] currPos = getPositionFromVumark();
        double[] currOri = getOrientationFromVumark();

        double headingOffset = headingOffset(currOri[2], targetOri[0]);
        double[] posOffset = posOffset(currPos, targetPos[0]);

        turn(-headingOffset,0.3, THRESHOLD_90);
        moveInches(-posOffset[1],-posOffset[0],0.5);

        while (!targetVisible)
        {
            for (VuforiaTrackable trackable : allTrackables)
            {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null)
                    {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            // Provide feedback as to where the robot is located (if we know).
            moveInches(-2.5, 0, 0.3);
            telemetry.addData("Visible Target", "none");
        }

        currPos = getPositionFromVumark();
//        currOri = getOrientationFromVumark();
//        headingOffset = headingOffset(currOri, targetOriTwo);
        posOffset = posOffset(currPos, targetPos[1]);
        telemetry.addData("XOffset", posOffset[0]);
        telemetry.addData("YOffset", posOffset[1]);
        telemetry.addData("HeadingOffset", headingOffset);
//        turn(-headingOffset, THRESHOLD_90);
        moveInches(-posOffset[1],-posOffset[0],0.5);

        moveInches(0,70,0.5);
        telemetry.update();

        requestOpModeStop();
    }
}
