/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SensorMRRangeSensor} illustrates how to use the Modern Robotics
 * Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
//@Disabled comment out or remove this line to enable this opmode
public class Ultra2Test extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensor1, rangeSensor2;
    List<Double> sen1 = new ArrayList<Double>();
    List<Double> sen2 = new ArrayList<Double>();

    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensor1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "s1");
        rangeSensor2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "s2");

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("1: ", "%.2f cm", rangeSensor1.getDistance(DistanceUnit.CM));
            telemetry.addData("2: ", "%.2f cm", rangeSensor2.getDistance(DistanceUnit.CM));

            if(this.gamepad1.left_bumper){
                while(this.gamepad1.left_bumper);
                sen1.add(rangeSensor1.getDistance(DistanceUnit.CM));
            }
            if(this.gamepad1.right_bumper){
                while(this.gamepad1.right_bumper);
                sen2.add(rangeSensor2.getDistance(DistanceUnit.CM));
            }
            if(this.gamepad1.a){
                while(this.gamepad1.a);
                sen1.add(rangeSensor1.getDistance(DistanceUnit.CM));
                sen2.add(rangeSensor2.getDistance(DistanceUnit.CM));
            }

            telemetry.addData("Sensor 1 Data: ", sen1);
            telemetry.addData("Sensor 2 Data: ", sen2);
            telemetry.update();
        }
    }
}

