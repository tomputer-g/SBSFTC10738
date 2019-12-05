/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode20.BaseAuto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaSkystoneChaserWebcam extends BaseAuto {

    private static double distGoal = 0;
    private double[] displacements = {2, 7};//+ = forward; + = right
    private double headingDisplacement = -90;
    private ElapsedTime t;
    private int position = -1;
    //rotate -90 for heading


    @Override
    public void init() {
        telemetry.setAutoClear(false);
        initVuforiaWebcam();
        initDrivetrain();
        initIMU();
    }


    @Override
    public void loop() {//range 17in
        if(t == null){
            moveInchesG(0,12,0.4);
            telemetry.clear();
            t = new ElapsedTime();
            targetsSkyStone.activate();
        }
        boolean targetVisible = false;
        if(position == -1) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    //telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    if (trackable.getName().equals("Stone Target")) {
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addLine("Turn " + (int) Math.abs(rotation.thirdAngle + headingDisplacement) + (rotation.thirdAngle + headingDisplacement > 0 ? "deg. CW" : "deg. CCW"));
                        VectorF translation = lastLocation.getTranslation();
                        double dist = translation.get(1) / mmPerInch + displacements[1];
                        telemetry.addLine("Move " + Math.abs(dist) + (dist > 0 ? "in. Right" : "in. Left"));
                        if (dist > 5) {
                            position = 2;
                            telemetry.addData("Capture time", t.milliseconds());
                            telemetry.addData("Position", position);
                        } else if (dist > -5) {
                            position = 1;
                            telemetry.addData("Capture time", t.milliseconds());
                            telemetry.addData("Position", position);
                        }
                        telemetry.addLine("Forward " + (distGoal - translation.get(0) / mmPerInch + displacements[0]) + "in.");
                        //moveInches(translation.get(1)/mmPerInch+displacements[1], translation.get(0)/mmPerInch+displacements[0], 0.4);
                    }
                    break;
                } else if (t.milliseconds() > 500) {
                    telemetry.addLine("Exceeded 500ms wait.");
                    position = 0;
                    telemetry.addData("Position", position);
                }
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        targetsSkyStone.deactivate();
    }
}
