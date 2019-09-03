package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Disabled
public class CRServoTest extends OpMode {
    private CRServo servo1;
    private CRServo servo2;

    @Override
    public void init() {
        servo1 = hardwareMap.get(CRServo.class,"servo1");
        servo2 = hardwareMap.get(CRServo.class,"servo2");
    }

    @Override
    public void loop() {
        if(this.gamepad1.left_bumper){
            while(this.gamepad1.left_bumper){
                servo1.setPower(1);
                servo2.setPower(-1);
            }

        }else if(this.gamepad1.right_bumper){
            while(this.gamepad1.right_bumper) {
                servo1.setPower(-1);
                servo2.setPower(1);
            }
        }
        if(this.gamepad1.x){
            servo1.setPower(0);
            servo2.setPower(0);
        }
    }
}
