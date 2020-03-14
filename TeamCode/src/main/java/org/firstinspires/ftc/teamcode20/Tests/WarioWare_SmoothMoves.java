package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.openftc.revextensions2.ExpansionHubEx;

import java.lang.invoke.VolatileCallSite;

@TeleOp
public class WarioWare_SmoothMoves extends BaseAuto {

    private double[] params = {0,0, -50};
    private String[] paramNames = {"P", "D","set Inch/s"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb, a;
    private boolean runOdoSpeed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initIMU();
        initOdometry();
        initLogger("Smoothmoves-"+ System.currentTimeMillis()+".csv");
        OdometrySpeedThread odometrySpeedThread = new OdometrySpeedThread();
        odometrySpeedThread.start();
        runOdoSpeed = false;
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a){a = true;}if(!this.gamepad1.a && a){
                a = false;
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
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] - 1) * 1E6) / 1E6;

            }
            if(this.gamepad1.dpad_right){r = true;}if(!this.gamepad1.dpad_right && r){
                r = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] + 1) * 1E6) / 1E6;

            }
            if(this.gamepad1.dpad_up){u = true;}if(!this.gamepad1.dpad_up && u){
                u = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] * 10.0) * 1E6) / 1E6;

            }
            if(this.gamepad1.dpad_down){d = true;}if(!this.gamepad1.dpad_down && d){
                d = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] / 10.0) * 1E6) / 1E6;

            }

            StringBuilder paramDisplay= new StringBuilder();
            for(double d : params){
                paramDisplay.append(d);
            }
            odometrySpeedThread.setVOC = params[2] * odometryEncYPerInch;
            telemetry.addData("params",paramDisplay);
            telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            telemetry.addData("Running?",runOdoSpeed);
            telemetry.addData("setVOC", odometrySpeedThread.setVOC);
            telemetry.addData("Actual", odometrySpeedThread.lastSpeed);
            telemetry.addData("VOC counter", odometrySpeedThread.VOCReachCounter);
            telemetry.addData("VOC reached?", odometrySpeedThread.VOCReached);
            telemetry.addData("Y1/Y2",""+getY1Odometry()+"/"+getY2Odometry());
            telemetry.update();
        }
        odometrySpeedThread.stop = true;
        stopLog();
        setAllDrivePower(0);
    }


    private class OdometrySpeedThread extends Thread{
        volatile boolean stop = false;
        public double setPower = 0;
        private ElapsedTime t;
        private int lastPosition;
        public double lastSpeed;
        private double setVOC = 0;
        boolean VOCReached = false;
        private int VOCReachCounter = 0;
        private double kP = 1E-6, kD = 9E-6;


        @Override
        public void run() {
            Log.d("OdometrySpeed"+this.getId(),"Started running");
            t = new ElapsedTime();
            lastPosition = getXOdometry();//getY1Odometry();
            int currentOdo = lastPosition;
            writeLog("ns,position,kPX,kDX,targetSpeed,lastSpeed,dPower,setPower,"+hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2").read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)+"V");
            while(!isInterrupted() && !stop){
                if(runOdoSpeed){
                    lastPosition = currentOdo;
                    currentOdo = getXOdometry();//getY1Odometry();
                    long ns = t.nanoseconds();
                    t.reset();
                    double currentSpeed = (currentOdo-lastPosition)*1.0E9/ns;
                    double dPower = kP * (setVOC - currentSpeed) + kD * (lastSpeed - currentSpeed);
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
                    writeLog(""+ns+","+currentOdo+","+kP+","+kD+","+setVOC+","+lastSpeed+","+dPower+","+setPower);

                }else{
                    setAllDrivePower(0);
                    t.reset();
                    currentOdo = lastPosition;
                    lastPosition = getXOdometry();//getY1Odometry();
                    lastSpeed = 0;
                    setPower = 0;
                }
                if(near(lastSpeed, setVOC, odometryEncYPerInch)){// flag for if speed reached
                    if(VOCReachCounter < 5)VOCReachCounter++;
                    if(VOCReachCounter > 5)VOCReached = true;

                }else{
                    VOCReachCounter = 0;
                    VOCReached = false;
                }
            }
            Log.d("OdometrySpeed"+this.getId(), "finished thread");

        }
    }



}
