package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode19.BaseAuto;
@Disabled
@TeleOp
public class VumarkTargetHeadingCalibrate extends BaseAuto {
        double AngleLocked;
        double speed;
        double threshold;
        double turningAmount;

        public void init(){
            useTFOD = false;
            useVuMark = true;
            super.init();
            AngleLocked = 90;
            speed = 0.2;
            threshold = 2;
            turningAmount = 0;
        }

    @Override
    public void init_loop() {

    }

    public void loop(){
            stopTFOD();
            if (this.gamepad1.dpad_left)
            {
                while (this.gamepad1.dpad_left);
                AngleLocked -= 1;
            }
            else if (this.gamepad1.dpad_right)
            {
                while(this.gamepad1.dpad_right);
                AngleLocked += 1;
            }
            else if (this.gamepad1.dpad_up)
            {
                while (this.gamepad1.dpad_up);
                speed += 0.1;
            }
            else if (this.gamepad1.dpad_down)
            {
                while(this.gamepad1.dpad_down);
                speed -= 0.1;
            }
            else if (this.gamepad1.left_bumper)
            {
                while(this.gamepad1.left_bumper);
                threshold-=1;
            }
            else if (this.gamepad1.right_bumper)
            {
                while(this.gamepad1.right_bumper);
                threshold+=1;
            }
            else if (this.gamepad1.x)
            {
                while(this.gamepad1.x);
                AngleLocked = -AngleLocked;
            }
            else if (this.gamepad1.y)
            {
                while(this.gamepad1.y);

                waitUntilVumarkRecognized();
                telemetry.clearAll();
                telemetry.addLine("OK");
                telemetry.update();
                while(!this.gamepad1.a);
                while(this.gamepad1.a);
                refreshVumarkReading();
                telemetry.update();

                while(!this.gamepad1.a);
                while(this.gamepad1.a);
                double[] currOrientation = getOrientationFromVumark();
                double currHeading = currOrientation[2];
                turningAmount = headingOffset(currHeading, AngleLocked);
                turn(turningAmount, speed, threshold);
            }
            telemetry.addData("Current speed: ","%.2f", speed);
            telemetry.addData("Angle Locked: ", AngleLocked);
            telemetry.addData("Current threshold: ", threshold);
            telemetry.addData("Last Turning Amount: ", turningAmount);
            telemetry.update();
        }
}
