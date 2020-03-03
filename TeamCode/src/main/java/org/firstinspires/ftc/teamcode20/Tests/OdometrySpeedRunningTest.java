package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.openftc.revextensions2.ExpansionHubEx;
@TeleOp
public class OdometrySpeedRunningTest extends BaseAuto {
    private boolean runOdoSpeed = false, BPrimed = false;
    private OdometrySpeedThread odometrySpeedThread;
    private double[] params =       {1E-6,  9E-6,  -10000, 1};//opt: 1E-6, 3E-5, ...,1
    private String[] paramNames =   {"kP",   "kD", "targetSpeed", "k"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb;

    protected final double odometryEncPerInch = 1324.28;


    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initOdometry();
        initLogger("OdoSpeedTest"+System.currentTimeMillis()+".csv");
        odometrySpeedThread = new OdometrySpeedThread();
        odometrySpeedThread.start();
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed) {
                BPrimed = false;
                runOdoSpeed = !runOdoSpeed;
            }
            if(this.gamepad1.left_bumper){lb = true;}if(!this.gamepad1.left_bumper && lb){
                lb = false;
                currentSelectParamIndex--;
                if(currentSelectParamIndex < 0){
                    currentSelectParamIndex = params.length - 1;
                }
            }
            if(this.gamepad1.right_bumper){rb = true;}if(!this.gamepad1.right_bumper && rb){
                rb = false;
                currentSelectParamIndex++;
                if(currentSelectParamIndex >= params.length){
                    currentSelectParamIndex = 0;
                }
            }
            if(this.gamepad1.dpad_left){l = true;}if(!this.gamepad1.dpad_left && l){
                l = false;
                if(currentSelectParamIndex == 2){
                    params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] - 1E4) * 1E9) / 1E9;
                }else {
                    params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] - 1E-6) * 1E9) / 1E9;
                }
            }
            if(this.gamepad1.dpad_right){r = true;}if(!this.gamepad1.dpad_right && r){
                r = false;
                if(currentSelectParamIndex == 2){
                    params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] + 1E4) * 1E9) / 1E9;
                }else {
                    params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] + 1E-6) * 1E9) / 1E9;
                }
            }
            if(this.gamepad1.dpad_up){u = true;}if(!this.gamepad1.dpad_up && u){
                u = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] * 10.0) * 1E9) / 1E9;

            }
            if(this.gamepad1.dpad_down){d = true;}if(!this.gamepad1.dpad_down && d){
                d = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] / 10.0) * 1E9) / 1E9;

            }

            telemetry.addData("parameters",params[0]+", "+params[1]+", "+params[2]);
            telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            if(runOdoSpeed)telemetry.addLine("Odometry speed running");
            telemetry.addData("Actual speed (enc/s)",odometrySpeedThread.lastSpeed);
            telemetry.addData("Actual speed (power)",odometrySpeedThread.setPower);
            telemetry.update();
        }
        stopLog();
    }






    private class OdometrySpeedThread extends Thread{
        volatile boolean stop = false;
        public double setPower = 0;
        private ElapsedTime t;
        private int lastPosition;
        public double lastSpeed;


        @Override
        public void run() {
            Log.d("OdometrySpeed"+this.getId(),"Started running");
            t = new ElapsedTime();
            lastPosition = getXOdometry();//getY1Odometry();
            int currentOdo = lastPosition;
            writeLogHeader("ns,position,kPX,kDX,targetSpeed,lastSpeed,dPower,setPower,"+hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2").read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)+"V");
            while(!isInterrupted() && !stop){
                if(runOdoSpeed){
                    lastPosition = currentOdo;
                    currentOdo = getXOdometry();//getY1Odometry();
                    long ns = t.nanoseconds();
                    t.reset();
                    double currentSpeed = (currentOdo-lastPosition)*1.0E9/ns;
                    double dPower = params[3]*(params[0] * (params[2] - currentSpeed) + params[1] * (lastSpeed - currentSpeed));
                    setPower += dPower;
                    setPower = Math.min(1, Math.max(-1, setPower));
                    if(near(currentSpeed,0,10) && t.milliseconds() < 1000){
                        setPower = Math.signum(setPower) * 0.4;//no
                    }
                    lastSpeed = currentSpeed;
                    LF.setPower(-setPower);
                    LB.setPower(setPower);
                    RF.setPower(-setPower);
                    RB.setPower(setPower);
                    writeLog(""+ns+","+currentOdo+","+params[0]+","+params[1]+","+params[2]+","+lastSpeed+","+dPower+","+setPower);

                }else{
                    setAllDrivePower(0);
                    t.reset();
                    currentOdo = lastPosition;
                    lastPosition = getXOdometry();//getY1Odometry();
                    lastSpeed = 0;
                    setPower = 0;
                }
            }
            Log.d("OdometrySpeed"+this.getId(), "finished thread");

        }
    }

}
