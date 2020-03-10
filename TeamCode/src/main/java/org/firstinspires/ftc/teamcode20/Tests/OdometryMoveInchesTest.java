package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.openftc.revextensions2.ExpansionHubEx;
@TeleOp
public class OdometryMoveInchesTest extends BaseAuto {
    /*
    target is 100 inches

    0.3 speed: P = 1,       D = 0.12,   result: +1/32 in
    0.6 speed: P = 0.075,   D = 1.4E-2, result: 0 in.
    0.9 speed: P = 0.0325,  D = 7.3E-3, result: spin +- 1/4 in
     */
    private double[] params =       {0.0325,  0,     7.3E-3};//,  0.9,        90,           0.915};
    private String[] paramNames =   {"P",   "I",    "D"};//,    "speed",    "targetInches","k"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb, y, APrimed = false, x = false, platformGrabbed = false;

    private double displayMS = 0;
    protected final double odometryEncYPerInch = 1324.28, odometryEncXPerInch = 1316.38;

    @Override
    public void runOpMode() throws InterruptedException {
        initHubs();
        initDrivetrain();
        initPlatformGrabber();
        initOdometry();
        initIMU();
        initLogger("OdoMoveInchesX"+System.currentTimeMillis()+".csv");
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
                resetY1Odometry();
                resetXOdometry();
                setNewGyro0();
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ElapsedTime t = new ElapsedTime();
                //moveInchesYSpin(25,90,0.5);
                //moveInchesGOY_XFixed(90,0.9,1,getXOdometry()); 0.96 deg in 2.35s
                displayMS = t.milliseconds();
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

            if(this.gamepad1.x) {
                x = true;
            }if(!this.gamepad1.x && x){
                x = false;
                if(platformGrabbed){//already held
                    platformGrabbed = false;
                    platform_grabber.setPower(1);
                    Thread.sleep(100);
                    platform_grabber.setPower(0);
                }else{
                    platformGrabbed = true;
                    platform_grabber.setPower(-0.4);
                }
            }
            telemetry.addData("parameters",params[0]+", "+params[1]+","+params[2]);
            telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            telemetry.addData("enc X", getXOdometry());
            telemetry.addData("enc Y1",getY1Odometry());
            telemetry.addData("imu",getHeading());
            telemetry.addData("last run time",displayMS);
            telemetry.update();
        }
        stopLog();
    }

    protected void setAllDrivePowerPolar(double r, double theta){
        LF.setPower(r* (-Math.sin(theta)-Math.cos(theta)));
        LB.setPower(r* (Math.sin(theta)-Math.cos(theta)));
        RF.setPower(r* (-Math.sin(theta)+Math.cos(theta)));
        RB.setPower(r* (Math.sin(theta)+Math.cos(theta)));
    }

    /*protected void moveInchesYSpin(double yInch, double angle, double speed,double a,double b,double c,double d,boolean resetOffset) throws InterruptedException {
        double turnkP = 0.068, turnkD = 0.9;
        double movekP = 0.1, movekD = 0.02;
        double turnSpd = 0.5, moveSpd = 0.5;

        //input processing
        yInch = -yInch;
        speed = Math.abs(speed);
        int steadyCounter = 0;if(resetOffset){
            acctarget=0;
            setNewGyro0();
        }
        double e = target;
        ElapsedTime t = new ElapsedTime();
        ElapsedTime n = new ElapsedTime();
        int i=0;
        while(i<5&&n.milliseconds()<((speed>0.5)?800:2000)){
            Thread.sleep(0);
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            t.reset();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(speed))P=P>0?speed:-speed;
            setAllDrivePower(P+D);
            e=e2;
            if(near(e2-e,0,0.3)&&near(e2,0,3))
                i++;
        }
        setAllDrivePower(0);
        acctarget+=target;
        if(resetOffset) {
            acctarget = 0;
            setNewGyro0();
        }
        else
            setNewGyro(acctarget);

        //shortcut because dumbass
    }
     */


    protected void moveInchesGOY_XFixed(double yInch, double speed,double kV, int FixXOffset) throws InterruptedException {//use 0.4 for short-dist
        yInch = -yInch;
        //setNewGyro0();
        double kP = params[0], kD = params[2];
        if(yInch == 0)return;
        //0.25 0.005
        double kPx = 0.2, kDx = 0.005;
        ElapsedTime t = new ElapsedTime();
        int offsetY = getY1Odometry();
        int offsetX = FixXOffset;
        double diff = 0;
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncYPerInch);
        double vy = speed;
        int previousPos = offsetY, previousPosX = offsetX, currentOdometry = offsetY, currentOdometryX, Dterm, DtermX;
        double tpre = 0, tcur;
        int steadyCounter = 0;

        double rampupCurrentSpeed = 0.2;
        while(rampupCurrentSpeed < 0.9){
            tcur = t.milliseconds();
            rampupCurrentSpeed += vy * 0.1;
            telemetry.addData("Rampup",rampupCurrentSpeed);
            telemetry.update();
            currentOdometryX = getXOdometry();

            setAllDrivePowerG(-rampupCurrentSpeed,-rampupCurrentSpeed,rampupCurrentSpeed,rampupCurrentSpeed,1.4);
            previousPosX = currentOdometryX;
            tpre = tcur;
        }
        while(steadyCounter < 3){
            //telemetry.addData("x",getXOdometry());
            //telemetry.addData("yL",getY1Odometry());
            //telemetry.addData("yR",getY2Odometry());
            //telemetry.update();
            Thread.sleep(0);

            currentOdometry = getY1Odometry();
            currentOdometryX = getXOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            DtermX = (int)((currentOdometryX - previousPosX)/(tcur-tpre));
            diff = (currentOdometryX - offsetX)/odometryEncXPerInch*kPx + (near(DtermX,0,speed * 5000 / 0.3)?(kDx *DtermX):0);
            multiply_factor = -Math.min(1, Math.max(-1, kV*((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vy,0.001) && near(prev_speed, 0, 0.1)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            Log.d("GOY "+yInch,"steady"+steadyCounter+", position"+currentOdometry+", LF speed"+prev_speed+", OC speed="+Dterm+"bulkSpd="+hub4.getBulkInputData().getMotorVelocity(platform_grabber));
            previousPos = currentOdometry;
            previousPosX = currentOdometryX;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*vy + diff,multiply_factor*vy - diff,multiply_factor*-vy + diff,multiply_factor*-vy - diff,1.4);
            prev_speed = multiply_factor * vy;
        }
        setAllDrivePower(0);
    }

}
