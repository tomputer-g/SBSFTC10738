package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;


public class AdaptiveOdometryPIDTest extends BaseAuto {

    /*max speed at 13.8V: -155,000 OC/s or 117in. going forward (bulk and normal agree there)
    double max = 0, min = 0;
    double bulkMax = 0, bulkMin = 0;
    int lastPosition = 0;

     */
    RevBulkData bulk;
    //ElapsedTime t;

    private double[] params =       {0.033,    0.0078,   60,             90,               1E-6,        0,          1};//9E-6
    private String[] paramNames =   {"P",  "D",    "speed_inch/s",    "targetInches",   "speed_kP", "speed_kD", "k"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb, y, APrimed = false, x = false;
    private boolean runOdoSpeed = true;

    protected final double odometryEncYPerInch = 1324.28, odometryEncXPerInch = 1316.38;

    private OdometrySpeedThread odoThread;


    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initOdometry();
        initIMU();
        initHubs();
        initLogger("AdaptOdoPID"+System.currentTimeMillis()+".csv");
        odoThread = new OdometrySpeedThread();
        odoThread.start();
        //t = new ElapsedTime();


        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
                resetY1Odometry();
                resetXOdometry();
                setNewGyro0();
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                moveInchesGOY_OCSpeed(params[3], params[2] * odometryEncYPerInch);
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
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] - 1) * 1E9) / 1E9;

            }
            if(this.gamepad1.dpad_right){r = true;}if(!this.gamepad1.dpad_right && r){
                r = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] + 1) * 1E9) / 1E9;

            }
            if(this.gamepad1.dpad_up){u = true;}if(!this.gamepad1.dpad_up && u){
                u = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] * 10.0) * 1E9) / 1E9;

            }
            if(this.gamepad1.dpad_down){d = true;}if(!this.gamepad1.dpad_down && d){
                d = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] / 10.0) * 1E9) / 1E9;

            }
            if(this.gamepad1.y){y = true;}if(!this.gamepad1.y && y){
                y=false;
                params[currentSelectParamIndex] = -params[currentSelectParamIndex];
            }

            telemetry.addData("parameters",params[0]+", "+params[1]+", "+params[2]+", "+params[3]+", "+params[4]+", "+params[5]+","+params[6]);
            telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            telemetry.addData("enc X", getXOdometry());
            telemetry.addData("enc Y1",getY1Odometry());
            telemetry.update();
        }

        odoThread.stop = true;
        stopLog();
    }


    private void moveInchesGOY_OCSpeed(double yInch, double OCSpeed){
        double kP = params[0], kD = params[1];
        yInch = -yInch;
        setNewGyro0();
        OCSpeed = Math.abs(OCSpeed);
        double multiply_factor, prev_speed = 0;
        int previousPos = getY1Odometry(), currentOdometry, Dterm, steadyCounter = 0, odometryYGoal = previousPos + (int)(yInch * odometryEncYPerInch);
        double tpre = 0, tcur;
        ElapsedTime t = new ElapsedTime();
        runOdoSpeed = true;
        writeLogHeader(""+hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)+",P "+kP+", D "+kD+", OCSpd "+OCSpeed+", yInch "+yInch+", k "+params[6]);
        writeLogHeader("time,,position,setPos,,OCSpeed,setSpeed,,actualSpeed(Thread),setPower*50k,setPower,,steadyCounter");
        while(steadyCounter < 5 && !this.gamepad1.b){
            currentOdometry = getY1Odometry();
            tcur = t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, params[6]*((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,OCSpeed * 5000 / (0.3 * 100 * odometryEncYPerInch))?(kD * Dterm):0))));
            previousPos = currentOdometry;
            tpre=tcur;
            odoThread.targetSpeed = multiply_factor * OCSpeed;
            prev_speed = odoThread.targetSpeed;
            telemetry.addData("mult_factor", multiply_factor);
            telemetry.addData("setting speed",odoThread.targetSpeed);
            telemetry.addData("current speed",odoThread.lastSpeed);
            telemetry.addData("distance",odometryYGoal-currentOdometry);
            telemetry.update();
            if(near(prev_speed, 0, odometryEncYPerInch * 0.5) && near(odometryYGoal, currentOdometry, odometryEncYPerInch)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            writeLog(""+tcur+",,"+currentOdometry+","+odometryYGoal+",,"+OCSpeed+","+odoThread.targetSpeed+",,"+odoThread.lastSpeed+","+odoThread.setPower*50000+","+odoThread.setPower+",,"+steadyCounter);
        }
        runOdoSpeed = false;
    }

    private class OdometrySpeedThread extends Thread{
        volatile boolean stop = false;
        public double setPower = 0;
        private ElapsedTime t;
        private int lastPosition;
        public double lastSpeed;
        public double targetSpeed = 0;

        @Override
        public void run() {
            Log.d("OdometrySpeed"+this.getId(),"Started running");
            t = new ElapsedTime();
            lastPosition = getY1Odometry();
            int currentOdo = lastPosition;
            while(!isInterrupted() && !stop){
                if(runOdoSpeed){
                    lastPosition = currentOdo;
                    currentOdo = getY1Odometry();
                    long ns = t.nanoseconds();
                    t.reset();
                    double currentSpeed = (currentOdo-lastPosition)*1.0E9/ns;
                    double dPower = (params[4] * (targetSpeed - currentSpeed) + params[5] * (lastSpeed - currentSpeed));
                    setPower += dPower;
                    setPower = Math.min(1, Math.max(-1, setPower));
                    lastSpeed = currentSpeed;
                    LF.setPower(setPower);
                    LB.setPower(setPower);
                    RF.setPower(-setPower);
                    RB.setPower(-setPower);

                }else{
                    setAllDrivePower(0);
                    t.reset();
                    currentOdo = lastPosition;
                    lastPosition = getY1Odometry();
                    lastSpeed = 0;
                    setPower = 0;
                }
            }
            Log.d("OdometrySpeed"+this.getId(), "finished thread");

        }
    }

    /*
    @Override
    public void loop() {
        if(this.gamepad1.a){
            setAllDrivePower(0,-1);
        }else if(this.gamepad1.x){
            setAllDrivePower(0,1);
        }else{
            setAllDrivePower(0);
        }
        if(this.gamepad1.b){
            min = 0;
            max = 0;
            bulkMax = 0;
            bulkMin = 0;
        }
        bulk = hub4.getBulkInputData();
        int bulkVel = bulk.getMotorVelocity(platform_grabber);
        long time = t.nanoseconds();
        t.reset();
        double speed = 1E9 * (bulk.getMotorCurrentPosition(platform_grabber) - lastPosition)/time;
        lastPosition = bulk.getMotorCurrentPosition(platform_grabber);
        bulkMin = Math.min(bulkMin, bulkVel);
        bulkMax = Math.max(bulkMax, bulkVel);
        min = Math.min(min, speed);
        max = Math.max(max, speed);
        telemetry.addData("voltage",hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
        telemetry.addData("Normal","max: "+to3d(max)+", min:"+to3d(min));
        telemetry.addData("Bulkdata","max: "+to3d(bulkMax)+", min:"+to3d(min));
        telemetry.addData("Current","norm: "+to3d(speed)+", bulk: "+to3d(bulkVel));
        telemetry.update();

    }

     */
}
