package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode20.Roadrunner.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.revextensions2.ExpansionHubEx;

import static java.lang.Math.sqrt;

@TeleOp
public class MoveTest extends BaseAuto {
    private double speeed, speed,x,y, GYRO_kp, side_distance, kp,kd,moveInches_kP = 0.5,odometryEncPerInch =1316;
    private int offsetX = 0, offsetY = 0;
    private boolean[] qq = {true}, bF={true}, lF = {true}, e = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};
    private ElapsedTime t=new ElapsedTime();
    private double speedLF=0,speedLB=0,speedRF=0,speedRB=0;
    private double  kP = 0.5, kI = 0, kD = 0.0025;
    private FtcDashboard dashboard;
    int WaitingTime = 300;
    int steps = 20;
    double basespeed = 0.2;

    private PowerThread powerThread=new PowerThread();

    int dir;

    @Override
    public void runOpMode() throws InterruptedException {
        msStuckDetectInit = 3000000;
        speed=0.5;
        speeed=1;
        dir=1;
        y = -90;
        x = 0;
        // 三天之内刹了你();
        initDrivetrain();
        initHubs();
        //initVuforia();
        //initViewMarks();
        initIMU();
        //drive=new SampleMecanumDriveREV(hardwareMap);
        initOdometry();
        //cooThread.start();
        //initVuforia();
        //initViewMarks();
        initIMU();
        initOdometry();
        initLogger("N.csv");
        //drive=new SampleMecanumDriveREV(hardwareMap)
        //cooThread.start();
        waitForStart();
        int inchh = 8;
        int powerr = 3; double basee = 1.0;int indicator = 0;
        while(!this.gamepad1.start) {
            if(zheng(this.gamepad1.dpad_up, e))powerr++;
            if(zheng(this.gamepad1.dpad_down, eee))powerr--;
            if(zheng(this.gamepad1.dpad_left, ee))basee-=0.1;
            if(zheng(this.gamepad1.dpad_right, ff))basee+=0.1;
            if(zheng(this.gamepad1.x,fff)) indicator++;

            telemetry.addData("num","%.2f",basee);
            telemetry.addData("power","%.2f",powerr);

            if(indicator%3==0)telemetry.addData("KP",0);
            else if(indicator%3==1)telemetry.addData("KI",0);
            else telemetry.addData("KD",0);

            if(zheng(this.gamepad1.right_bumper,bF)){
                double a = 100000;
                while(t.milliseconds()<2900){
                    a = setAllDrivePowerO(-0.4,-0.4,.4,.4,a,1/2000, 1);
                }
                setAllDrivePower(0);
            }

            /*
            if(zheng(this.gamepad1.right_bumper,bF)){
                setAllDrivePower(-speed,-speed,speed,speed);
                ElapsedTime t=new ElapsedTime();
                double xpre=0,tpre=t.milliseconds();
                double v=0;
                while(t.milliseconds()<5000&&!this.gamepad1.b){
                    double tcur=t.milliseconds();
                    setAllDrivePowerG(-speed,-speed,speed,speed);
                    double xcur = getY1Odometry();
                    v=(xcur-xpre)/(tcur-tpre);
                    telemetry.addData("v",v);
                    telemetry.update();
                    xpre = xcur;
                    tpre=tcur;
                    speed=speed*x/5000;
                }
                setAllDrivePower(0);
                speed=0.5;
            }

             */
            if(zheng(this.gamepad1.left_bumper,lF)){
                ElapsedTime t=new ElapsedTime();
                while(t.milliseconds()<x) {
                    setAllDrivePowerG(-speed*f,-speed*f,speed*f,speed*f);
                }
                t.reset();
                while(t.milliseconds()<x-100){
                    setAllDrivePowerG(-0.7*f,-0.7*f,0.7*f,0.7*f);
                }
                while(t.milliseconds()<5000 && !this.gamepad1.b){
                    setAllDrivePowerG(-speeed*f,-speeed*f,speeed*f,speeed*f);
                }
                setAllDrivePower(0);
                f=-f;
            }
            //telemetry.addData("s",adjustToViewMark(true)[1]);
            //telemetry.addData("s",adjustToViewMark(false)[1]);
            //telemetry.addData("y",x/5000);
            //telemetry.addData("x",x);
            telemetry.addData("Imu",getHeading());
            //telemetry.addData("speeed",speeed);
            //telemetry.addData("speed", speed);
            telemetry.addData("Y1: ",getY1Odometry());
            telemetry.addData("Y2: ",getY2Odometry());
            telemetry.update();
        }
        //cooThread.stopThread();
    }

    private class PowerThread extends Thread{
        volatile public boolean stop = false;
        private double powera,powerb,powerc,powerd;

        public PowerThread(){
            powera=0;
            powerb=0;
            powerc=0;
            powerd=0;
        }
        public void setPower(double p){
            powera=p;
            powerb=p;
            powerc=p;
            powerd=p;
        }
        public void setPower(double a,double b,double c,double d){
            powera=a;
            powerb=b;
            powerc=c;
            powerd=d;
        }
        @Override
        public void run() {
            //this.setPriority(4);
            this.setName("Power Thread "+this.getId());
            while (!isInterrupted() && !stop) {
                if(powera!=0&&powerb!=0&&powerc!=0&&powerd!=0)setAllDrivePowerG(powera,powerb,powerc,powerd);
            }
        }
        public void stopThread(){
            stop = true;
        }
    }

    public void setAllDrivePowerG(double p){
        powerThread.setPower(p);
    }
    public void setAllDrivePowerG(double a,double b,double c,double d){
        powerThread.setPower(a,b,c,d);
    }
}
