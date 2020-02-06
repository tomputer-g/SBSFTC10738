package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode20.BaseAuto;

import static java.lang.Math.sqrt;

@TeleOp
public class MoveTest extends BaseAuto {
    private double speeed, speed,x,y, GYRO_kp, side_distance, kp,kd,moveInches_kP = 0.5,odometryEncPerInch =1316;
    private int offsetX = 0, offsetY = 0;
    private boolean[] qq = {true}, bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};
    private ElapsedTime t=new ElapsedTime();
    private double speedLF=0,speedLB=0,speedRF=0,speedRB=0;
    private double  kP = 0.5, kI = 0, kD = 0.0025;
    int WaitingTime = 300;
    int steps = 20;
    double basespeed = 0.2;


    private PG pg=new PG();
    private Thread uc=new UC();
    int dir;
    private void 三天之内刹了你(){
        setAllDrivePower(1,1,-1,-1);
        wait(200);
        setAllDrivePower(0);
    }

    @Override
    public void init(){
        msStuckDetectInit = 3000000;
        initAutonomous();
        //rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        speed=0.3;
        speeed = 0.03;
        dir=1;
        y = -90;
        x = 0;
        // 三天之内刹了你();
    }

    @Override
    public void start(){
        //pg.start();
        //uc.start();
    }

    @Override
    public void stop(){

    }

    @Override
    public void loop(){
        //if(zheng(this.gamepad1.dpad_left,eee))x-=2;
        if(zheng(this.gamepad1.dpad_right,fff))x+=2;
        if(zheng(this.gamepad1.dpad_up,ee))y+=1;
        if(zheng(this.gamepad1.dpad_down,ff))y+=1;
        if(zheng(this.gamepad1.y,m))speed+=.1;
        if(zheng(this.gamepad1.a,mm))speed-=.1;
        if(zheng(this.gamepad1.b,f))moveInchesGOY_XF(48,0.3,1);

        if(zheng(this.gamepad1.left_bumper,bF)){
            ElapsedTime t=new ElapsedTime();
            targetsSkyStone.activate();
            VuforiaTrackable trackable = allTrackables.get(4);
            while(t.milliseconds()<500000) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    if (trackable.getName().equals("Blue Front Bridge")) {
                        Orientation rotation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                        telemetry.addLine("Turn " + (int) Math.abs(rotation.thirdAngle - 90) + (rotation.thirdAngle - 90 > 0 ? "deg. CW" : "deg. CCW"));
                        VectorF translation = lastLocation.getTranslation();
                        double disty = translation.get(1)/mmPerInch;
                        double distx = translation.get(0)/mmPerInch;
                        double distz = translation.get(2)/mmPerInch;
                        telemetry.addData("x: ",distx);
                        telemetry.addData("y: ",disty);
                        telemetry.addData("z: ",distz);
                    }
                    telemetry.update();
                }
            }
            shutdownVuforia();
        }

        if(zheng(this.gamepad1.right_bumper,lF)) {
            int offsetX = getXOdometry();
            double diff = 0;
            ElapsedTime p = new ElapsedTime();
            p.reset();
            while (p.milliseconds()<3500){
                diff = (getXOdometry() - offsetX)/odometryEncXPerInch/12;
                setAllDrivePowerG(-.3 +diff,-.3 -diff ,.3 +diff ,.3 -diff);
            }
            setAllDrivePower(0);
        }

        if(zheng(this.gamepad1.dpad_left,e)) {
            int offsetY = getY1Odometry();
            double diff = 0;
            ElapsedTime p = new ElapsedTime();
            p.reset();
            while (p.milliseconds()<5500){
                diff = (getY1Odometry() - offsetY)/odometryEncYPerInch/12;
                setAllDrivePowerG(-.5 -diff,.5 -diff ,-.5 +diff ,.5 +diff);
            }
            setAllDrivePower(0);
        }

        /*
        telemetry.addData("x: ",x);
        telemetry.addData("y: ",y);
        //telemetry.addData("Imu: ","%.2f",getHeading());
        telemetry.addData("target: ",acctarget);
        telemetry.addData("Speed: ","%.2f" ,speed);
        //telemetry.addData("[x]: ","%.2f",n_pass[0]);
        //telemetry.addData("[y]: ","%.2f" ,n_pass[1]);;
        telemetry.update();


         */

    }

    private class UC extends Thread{
        volatile boolean stop = false,run=false;
        @Override
        public void run() {
            while(!isInterrupted()&&!stop){
                updateCoo();
            }
        }
        public void stopThread(){
            stop = true;
        }
    }

    private class PG extends Thread {
        volatile boolean stop = false, run = false;
        private double a, b, c, d, Kp;

        public void PG() {
            a = 0;
            b = 0;
            c = 0;
            d = 0;
            Kp = .8;
        }

        public void setAllPower(double w, double x, double y, double z) {
            a = w;
            b = x;
            c = y;
            d = z;
        }

        @Override
        public void run() {
            double p = 0;
            while (!isInterrupted() && !stop) {
                if (a == 0 && b == 0 && c == 0 && d == 0) {
                } else {
                    p = Kp * (getHeading() * 0.1 / 9);
                    setAllDrivePower(a - p, b - p, c - p, d - p);
                }
            }


        }
    }

    protected void tunePIDturn(double target, double kp, double kd, double speed){
        setNewGyro(acctarget);
        double e =getError(target);
        ElapsedTime t = new ElapsedTime();
        int i=0;
        while(i<5){
            double e2 = getError(target);
            double D = kd*(e2-e)/t.milliseconds();
            t.reset();
            double P = e2*kp;
            double power=P+D;
            if(power!=0)
                setAllDrivePower((power>0)?Range.clip(power,.2,speed):Range.clip(power,-speed,-.2));
            e=e2;
            if(near(e2-e,0,0.1)&&near(e,0,2))i++;
        }
        setAllDrivePower(0);
        acctarget=getError(acctarget+target,0);
    }
}

