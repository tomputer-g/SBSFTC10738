package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;
@Deprecated
//@Autonomous(group = "Test")
public class motorEncoderDisplay extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        msStuckDetectLoop = 15000;
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    @Override
    public void loop() {
        motor.setTargetPosition(1500);
        wait(5500);
        motor.setTargetPosition(-1500);
        wait(5500);

    }


    protected void wait(int time){
        try {sleep(time);} catch (InterruptedException e) {e.printStackTrace();}
    }
}
