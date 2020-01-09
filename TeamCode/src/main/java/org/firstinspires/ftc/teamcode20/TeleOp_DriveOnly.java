package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.BaseTeleOp;
@TeleOp
public class TeleOp_DriveOnly extends BaseAuto {
    private OdometryThread thread=new OdometryThread();

    @Override
    public void init() {
        initDrivetrain();
        LF.setPower(1);
    }

    @Override
    public void loop() {
        //for old bot only
        //move(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);
        //new bot
        move(-this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x));
    }

    @Override public void start() {
        super.start();
        thread.start();
    }

    @Override public void stop() {
        thread.stopThread();
        super.stop();
    }

    private class OdometryThread extends Thread{
        volatile boolean stop = false;
        @Override
        public void run() {
            while(!isInterrupted()&&!stop){
                updateCoo();
                if(telemetryOn)telemetry.addLine("(x,y): ("+coo[0]+","+coo[1]+")");
            }
        }
        public void stopThread(){
            stop = true;
        }
    }

    @Override
    protected void initDrivetrain() {
        super.initDrivetrain();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
