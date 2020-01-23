package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class OdometrySpeedRunningTest extends BaseAuto {
    private boolean runOdoSpeed = false, BPrimed = false, leftP = false, rightP = false, upP = false, downP = false;
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
            odometrySpeedThread.setTargetSpeed(odometrySpeedThread.targetSpeed + 100);
        }
        if(this.gamepad1.dpad_down){downP = true;}if(!this.gamepad1.dpad_down && downP){
            downP = false;
            odometrySpeedThread.setTargetSpeed(odometrySpeedThread.targetSpeed - 100);
        }
        if(this.gamepad1.dpad_left){leftP = true;}if(leftP && !this.gamepad1.dpad_left){ leftP = false;
            //odometrySpeedThread.kD /= 10.0;
            odometrySpeedThread.kPX /= 10.0;
        }
        if(this.gamepad1.dpad_right){rightP = true;}if(rightP && !this.gamepad1.dpad_right){ rightP = false;
            //odometrySpeedThread.kD *= 10.0;
            odometrySpeedThread.kPX *= 10.0;
        }
        if(runOdoSpeed)telemetry.addLine("Odometry speed running");
        telemetry.addData("Odo target speed",odometrySpeedThread.targetSpeed+"enc/s");
        telemetry.addData("kP", odometrySpeedThread.kPX);
        telemetry.addData("kD", odometrySpeedThread.kDX);
        telemetry.addData("Actual speed (enc/s)",odometrySpeedThread.lastSpeed);
        telemetry.addData("Actual speed (power)",odometrySpeedThread.setPower);
        telemetry.update();
    }


    private class OdometrySpeedThread extends Thread{
        volatile boolean stop = false;
        public int targetSpeed = -12000;
        public double setPower = 0;
        private ElapsedTime t;
        private int lastPosition;
        public double lastSpeed;
        public double kPY = 1E-6, kDY = 0, kPX = 1E-5, kDX = 0;


        @Override
        public void run() {
            Log.d("OdometrySpeed"+this.getId(),"Started running");
            t = new ElapsedTime();
            lastPosition = getYOdometry();
            //lastPosition = getXOdometry();
            writeLogHeader("ns,position,kPX,kDX,targetSpeed,lastSpeed,dPower,setPower");
            while(!isInterrupted() && !stop){
                if(runOdoSpeed){
                    int currentOdo = getYOdometry();
                    long ns = t.nanoseconds();
                    double currentSpeed = (currentOdo-lastPosition)*1.0E9/ns;
                    double dPower = (kPY * (targetSpeed - currentSpeed) + kDY * (currentSpeed - lastSpeed));
                    setPower += dPower;
                    setPower = Math.min(0, Math.max(-1, setPower));//0,-1
                    lastSpeed = (currentOdo-lastPosition)*1.0E9/ns;
                    setAllDrivePower(0,setPower);
                    //setAllDrivePower(setPower, 0);
                    t.reset();
                    writeLog(""+ns+","+currentOdo+","+kPY+","+kDY+","+targetSpeed+","+lastSpeed+","+dPower+","+setPower);
                    lastPosition = currentOdo;
                    //lastPosition = getXOdometry();

                }else{
                    setAllDrivePower(0);
                    t.reset();
                    lastPosition = getYOdometry();
                    //lastPosition = getXOdometry();
                    lastSpeed = 0;
                    setPower = 0;
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
