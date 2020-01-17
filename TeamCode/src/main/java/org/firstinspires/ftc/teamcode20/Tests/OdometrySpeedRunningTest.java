package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp
public class OdometrySpeedRunningTest extends BaseAuto {
    private boolean runOdoSpeed = true, BPrimed = false, leftP = false, rightP = false, upP = false, downP = false;
    private OdometrySpeedThread odometrySpeedThread;
    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
        initLogger("OdoSpeedTest"+System.currentTimeMillis()+".csv");
        odometrySpeedThread = new OdometrySpeedThread();
        odometrySpeedThread.start();
    }

    @Override
    public void loop() {
        if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed) {
            BPrimed = false;
            runOdoSpeed = !runOdoSpeed;
        }
        if(this.gamepad1.dpad_up){upP = true;}if(!this.gamepad1.dpad_up && upP) {
            upP = false;
            odometrySpeedThread.setTargetSpeed(odometrySpeedThread.targetSpeed + 50);
        }
        if(this.gamepad1.dpad_down){downP = true;}if(!this.gamepad1.dpad_down && downP){
            downP = false;
            odometrySpeedThread.setTargetSpeed(odometrySpeedThread.targetSpeed - 50);
        }
        if(this.gamepad1.dpad_left){leftP = true;}if(leftP && !this.gamepad1.dpad_left){ leftP = false;
            odometrySpeedThread.kD /= 10.0;
        }
        if(this.gamepad1.dpad_right){rightP = true;}if(rightP && !this.gamepad1.dpad_right){ rightP = false;
            odometrySpeedThread.kD *= 10.0;
        }
        if(runOdoSpeed)telemetry.addLine("Odometry speed running");
        telemetry.addData("Odo target speed",odometrySpeedThread.targetSpeed+"enc/s");
        telemetry.addData("kP", odometrySpeedThread.kP);
        telemetry.addData("kD", odometrySpeedThread.kD);
        telemetry.addData("Actual speed (enc/s)",odometrySpeedThread.lastSpeed);
        telemetry.addData("Actual speed (power)",odometrySpeedThread.setPower);
        telemetry.update();
    }

    private class OdometrySpeedThread extends Thread{
        volatile boolean stop = false;
        public int targetSpeed = -4000;
        public double setPower = 0;
        private ElapsedTime t;
        private int lastPosition;
        public double lastSpeed;
        public double kP = 1E-6, kD = 1E-6;

        @Override
        public void run() {
            Log.d("OdometrySpeed"+this.getId(),"Started running");
            t = new ElapsedTime();
            lastPosition = getYOdometry();
            writeLogHeader("kP,kD,ns,targetSpeed,lastSpeed,setPower");
            while(!isInterrupted() && !stop){
                if(runOdoSpeed){
                    long ns = t.nanoseconds();
                    double currentSpeed = (getYOdometry()-lastPosition)*1.0E9/ns;
                    //PID here
                    double dPower = (kP * (targetSpeed - currentSpeed) + kD * (currentSpeed - lastSpeed));
                    setPower += dPower;
                    setPower = Math.min(0, Math.max(-1, setPower));
                    lastSpeed = (getYOdometry()-lastPosition)*1.0E9/ns;
                    setAllDrivePower(0,setPower);
                    t.reset();
                    lastPosition = getYOdometry();
                    writeLog(""+kP+","+kD+","+ns+","+targetSpeed+","+lastSpeed+","+setPower);
                }else{
                    setAllDrivePower(0);
                    t.reset();
                    lastPosition = getYOdometry();
                }
            }
            Log.d("OdometrySpeed"+this.getId(), "finished thread");

        }

        public void setTargetSpeed(int speed){
            if(speed >= 0) {
                targetSpeed = speed;
            }
        }
    }

    @Override
    public void stop() {
        stopLog();
    }
}
