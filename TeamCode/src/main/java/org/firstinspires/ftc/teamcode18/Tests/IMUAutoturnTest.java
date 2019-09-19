package org.firstinspires.ftc.teamcode18.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode18.BaseOpMode;

/**
 * Created by Ziming Gao on 2/5/2018.
 */
@TeleOp(name = "REV_IMU_turn_test")
@Disabled
public class IMUAutoturnTest extends BaseOpMode {
    private int turnDeg;

    @Override
    public void init() {
        super.init();
        turnDeg = 90;
    }

    @Override
    public void loop() {
        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            turn(-turnDeg);
        }
        if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            turn(turnDeg);
        }
        if(this.gamepad1.dpad_down){
            while(this.gamepad1.dpad_down);
            turnDeg -= 5;
        }
        if(this.gamepad1.dpad_up){
            while(this.gamepad1.dpad_up);
            turnDeg += 5;
        }
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            resetAngle();
        }
        if(this.gamepad1.right_bumper){
            setAllDrivePower(0.1);
        }else if(this.gamepad1.left_bumper){
            setAllDrivePower(0.3);
        }else{
            setAllDrivePower(0);
        }
        telemetry.addData("angle",getAngle());
        telemetry.addData("turn angle",turnDeg);
        telemetry.update();
    }
}
