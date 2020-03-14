package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode20.BaseOpMode;
import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp
public class OldDummyBot extends BaseOpMode {
    Servo servo;
    boolean[] B = {false};
    private DcMotor old_LF, old_LB, old_RF, old_RB;
    @Override
    public void runOpMode() throws InterruptedException {
        old_LF = hardwareMap.get(DcMotor.class,"LF");
        old_LB = hardwareMap.get(DcMotor.class,"LB");
        old_RF = hardwareMap.get(DcMotor.class,"RF");
        old_RB = hardwareMap.get(DcMotor.class,"RB");
        old_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        old_LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        old_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        old_RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        old_LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        old_LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        old_RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        old_RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(.4);
        waitForStart();
        while(opModeIsActive()){
            if(checkButton(this.gamepad1.b,B)){
                if(servo.getPosition() < 0.2){
                    servo.setPosition(.4);
                }else{
                    servo.setPosition(0);
                }
            }
            move(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);
        }
    }

    protected void move(double vx, double vy, double vr){
        old_LF.setPower(-0.5 * (vx - vy + vr));
        old_LB.setPower(-0.5 * (-vy - vx + vr));
        old_RF.setPower(-0.5 * (vx + vy + vr));
        old_RB.setPower(-0.5 * (-vx + vy + vr));
    }



    protected static boolean checkButton(boolean b, boolean[] f){
        if(b||!f[0]){
            f[0]= !b;
            return f[0];
        }
        return false;
    }
}
