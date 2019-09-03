package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseOpMode;

@TeleOp
@Disabled
public class TestT extends BaseOpMode {
    private static ElapsedTime t = new ElapsedTime();
    private static int T = 150;
    @Override
    public void init() {
        super.init();
        setMode_RUN_WITH_ENCODER();
        telemetry.addLine("This accelerates the robot FORWARD. Please clear any obstacles. Press A when ready.");
        telemetry.update();
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            for(int i = 0;i <= T; i+= 10){
                move(-i / T);
            }
            move(-1);
            wait(1000);
            for(int i = T;i >= 0; i-= 10){
                move(-i / T);
            }
        }
        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            T -= 10;
        }else if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            T += 10;
        }
        telemetry.addData("T",T);
        telemetry.update();
    }


    private void move(double vy){
        LF.setPower(vy);
        LB.setPower(vy);
        RF.setPower(-vy);
        RB.setPower(-vy);
    }
}
