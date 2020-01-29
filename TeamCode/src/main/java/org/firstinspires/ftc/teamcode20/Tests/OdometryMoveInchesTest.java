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
    private double[] params =       {0.0325,  0,     7.3E-3,       0.9,        6,           };
    private String[] paramNames =   {"P",   "I",    "D",    "speed",    "targetInches"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb, y, APrimed = false, x = false, platformGrabbed = false;

    protected final double odometryEncYPerInch = 1324.28, odometryEncXPerInch = 1316.38;
    @Override
    public void init() {
        initDrivetrain();
        initPlatformGrabber();
        initOdometry();
        initIMU();
        initLogger("OdoMoveInchesX"+System.currentTimeMillis()+".csv");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2.setPhoneChargeEnabled(true);
    }
    @Override
    public void loop() {
        if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
            resetY1Odometry();
            resetXOdometry();
            setNewGyro0();
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            moveInchesGOY(params[4],params[3],params[0],params[2]);
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
                wait(100);//TODO: blocks thread?
                platform_grabber.setPower(0);
            }else{
                platformGrabbed = true;
                platform_grabber.setPower(-0.4);
            }
        }
        telemetry.addData("parameters",params[0]+", "+params[1]+", "+params[2]+", "+params[3]+", "+params[4]);
        telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
        telemetry.addData("enc X", getXOdometry());
        telemetry.addData("enc Y1",getY1Odometry());
        telemetry.addData("target", -params[4] * odometryEncYPerInch);
        telemetry.update();
    }


    @Override
    public void stop() {
        stopLog();
        super.stop();
    }

    protected void moveInchesGOY(double yInch, double speed,double kP,double kD){//use 0.4 for short-dist
        yInch = -yInch;
        setNewGyro0();
        ElapsedTime t = new ElapsedTime();
        int offsetY = getY1Odometry();
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncYPerInch);
        double vy = speed;
        int previousPos = offsetY, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        while(steadyCounter < 5 && !this.gamepad1.b){//b is there so we can break out of loop anytime
            currentOdometry = getY1Odometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, ((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vy,0.001) && near(currentOdometry, odometryYGoal, odometryEncYPerInch)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            Log.d("GOY "+yInch,"steady"+steadyCounter+", position"+currentOdometry+", speed"+prev_speed);
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*vy,multiply_factor*vy,multiply_factor*-vy,multiply_factor*-vy);
            prev_speed = multiply_factor * vy;
        }
        setAllDrivePower(0);

    }

    protected void moveInchesGOX_platform(double xInch, double speed){
        if(xInch == 0)return;
        writeLogHeader("P="+params[0]+", I="+params[1]+", D="+params[2]+",speed="+speed+",target="+xInch * odometryEncXPerInch+", batt"+hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
        writeLogHeader("millis time,position,P,eff. P Power,D,eff. D Power,LF speed,LF counts,GyroDrift");
        ElapsedTime t = new ElapsedTime();
        int offsetX = getXOdometry();
        speed=Math.abs(speed);
        double multiply_factor;
        int odometryXGoal = offsetX + (int)(xInch * odometryEncXPerInch);
        double vx = speed;
        int previousPos = offsetX, currentOdometry, Dterm;
        double tpre = 0, tcur;
        //antiSkidAccelerationX(0,vx,1000);
        while(!this.gamepad1.b){
            currentOdometry = getXOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, ((params[0] * (currentOdometry - odometryXGoal)/odometryEncXPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(params[2] * Dterm):0))));
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*-vx,multiply_factor*vx,multiply_factor*-vx,multiply_factor*vx);

            writeLog(t.milliseconds()+", "+currentOdometry+", "+((currentOdometry - odometryXGoal)/odometryEncXPerInch)+", "+((currentOdometry - odometryXGoal)/odometryEncXPerInch)*params[0]+", "+Dterm+", "+(near(Dterm,0,speed * 5000 / 0.3)?(params[2] * Dterm):"CLIPPED")+", "+LF.getPower()+", "+LF.getCurrentPosition()+", "+getHeading());
        }
        setAllDrivePower(0);
        writeLogHeader("Gyro drift="+getHeading()+", Ydrift="+ getY1Odometry());
        writeLogHeader("----End of GOX----");
    }
/*
    protected void moveInchesGOY(double yInch, double speed){
        yInch = -yInch;
        setNewGyro0();
        double kP = params[0], kD = params[2];
        if(yInch == 0)return;

        ElapsedTime t = new ElapsedTime();
        int offsetY = getY1Odometry();
        speed=Math.abs(speed);
        double multiply_factor, prev_speed = 0;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncYPerInch);
        double vy = speed;
        int previousPos = offsetY, currentOdometry, Dterm;
        double tpre = 0, tcur;
        int steadyCounter = 0;
        while(steadyCounter < 5 && !this.gamepad1.b){//b is there so we can break out of loop anytime
            currentOdometry = getY1Odometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            multiply_factor = -Math.min(1, Math.max(-1, ((kP * (currentOdometry - odometryYGoal)/ odometryEncYPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(kD * Dterm):0))));
            if(near(prev_speed, multiply_factor*vy,0.001) && near(currentOdometry, odometryYGoal, odometryEncYPerInch)){
                steadyCounter++;
            }else{
                steadyCounter = 0;
            }
            Log.d("GOY "+yInch,"steady"+steadyCounter+", position"+currentOdometry+", speed"+prev_speed);
            previousPos = currentOdometry;
            tpre=tcur;
            setAllDrivePowerG(multiply_factor*vy,multiply_factor*vy,multiply_factor*-vy,multiply_factor*-vy);
            prev_speed = multiply_factor * vy;
        }
        setAllDrivePower(0);

    }


 */
    private void antiSkidAccelerationX(double start, double goal, double accTime){
        writeLogHeader("start="+start+",goal="+goal+",acc time="+accTime+",batt "+hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2").read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)+"V");
        writeLogHeader("time,delta,current,odometry,wheel");
        ElapsedTime t = new ElapsedTime();
        double duration = accTime * Math.abs(goal - start), delta;
        setNewGyro0();
        double currentPower = start;
        setAllDrivePowerG(-currentPower,currentPower,-currentPower,currentPower);
        wait(100);
        t.reset();
        while(!this.gamepad1.b){
            delta = t.milliseconds()/duration;
            currentPower  = start + delta;
            if(currentPower > goal){
                break;
            }
            setAllDrivePowerG(-currentPower,currentPower,-currentPower,currentPower);
            writeLog(t.milliseconds()+","+delta+","+currentPower+","+getXOdometry()+","+LF.getCurrentPosition());
        }
        writeLogHeader("---AntiSlipAcc ended---");
    }
}
