package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.BaseTeleOp;

@Disabled
public class TeleOp_DriveOnly extends BaseAuto {
    private OdometryThread thread = new OdometryThread();

    @Override
    public void init() {
        initIMU();
        initDrivetrain();
        initOdometry();
        //LF.setPower(1);
    }

    @Override
    public void loop() {
        //for old bot only
        //move(-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);
        //new bot
        updateCoo();
        telemetry.addLine("(x,y): (" + (int)(coo[0]) + "," + (int)(coo[1]) + ")");
        telemetry.update();
        joystickScaledMove(-this.gamepad1.left_stick_x, -this.gamepad1.left_stick_y, (this.gamepad1.left_bumper ? 0 : -this.gamepad1.right_stick_x));
    }

    @Override
    public void start() {
        super.start();
        //thread.start();
    }

    @Override
    public void stop() {
        thread.stopThread();
        super.stop();
    }

    private class OdometryThread extends Thread {
        volatile boolean stop = false;

        @Override
        public void run() {
            while (!isInterrupted() && !stop) {

            }
        }

        public void stopThread() {
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

    protected void joystickScaledMove(double vx, double vy, double vr) {
        if (Math.abs(vx) > 0.2 || Math.abs(vy) > 0.2 || Math.abs(vr) > 0.2) {//deadzone
            double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
            double absMax = 0;
            for (double d : speeds)
                absMax = Math.max(Math.abs(d), absMax);
            if (absMax <= 1) {
                setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
            } else {
                if (showTelemetry) telemetry.addLine("SCALED power: max was " + absMax);
                setAllDrivePower(speeds[0] / absMax, speeds[1] / absMax, speeds[2] / absMax, speeds[3] / absMax);
            }
        } else setAllDrivePower(0);
    }
}
//have you heARD OF RAID SHADOW LEGENDS?