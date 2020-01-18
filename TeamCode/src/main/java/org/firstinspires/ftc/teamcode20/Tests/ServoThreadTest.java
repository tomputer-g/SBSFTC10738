package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseOpMode;
@Disabled

@TeleOp
public class ServoThreadTest extends BaseOpMode {


    @Override
    public void init() {
        initDrivetrain();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initGrabber();
        servoThread.setTarget(1);
    }

    @Override
    public void loop() {
        //for old bot only
        move(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);

        if(this.gamepad1.a){
            servoThread.setTarget(0.3);
        }
        if(this.gamepad1.b){
            servoThread.setTarget(1);
        }
        if(this.gamepad1.x){
            servoThread.setDelay(10);
        }
        if(this.gamepad1.y){
            servoThread.setDelay(30);
        }
        telemetry.addData("target", servoThread.targetPosition);
        telemetry.addData("last pos", servoThread.lastPosition);
        telemetry.addData("delay", servoThread.delayStep);
        telemetry.update();
    }

    @Override
    public void stop() {
        servoThread.stopThread();
        super.stop();
    }
}
