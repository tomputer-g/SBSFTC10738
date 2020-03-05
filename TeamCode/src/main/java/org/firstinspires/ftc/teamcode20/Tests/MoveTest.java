package org.firstinspires.ftc.teamcode20.Tests;

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
    private boolean[] qq = {true}, bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};
    private ElapsedTime t=new ElapsedTime();
    private double speedLF=0,speedLB=0,speedRF=0,speedRB=0;
    private double  kP = 0.5, kI = 0, kD = 0.0025;
    private FtcDashboard dashboard;
    int WaitingTime = 300;
    int steps = 20;
    double basespeed = 0.2;


    int dir;

    @Override
    public void runOpMode() throws InterruptedException {
        msStuckDetectInit = 3000000;
        speed=0.3;
        speeed = 0.03;
        dir=1;
        y = -90;
        x = 0;
        // 三天之内刹了你();
        initDrivetrain();
        initHubs();
        //initVuforia();
        //initViewMarks();
        initIMU();
        drive=new SampleMecanumDriveREV(hardwareMap);
        initOdometry();
        cooThread.start();
        initVuforia();
        //initViewMarks();
        initIMU();
        initOdometry();
        //drive=new SampleMecanumDriveREV(hardwareMap);
        //cooThread.start();
        waitForStart();
        int inchh = 8;
        while(!this.gamepad1.b) {
            if(zheng(this.gamepad1.dpad_left,eee))x-=2;
            if(zheng(this.gamepad1.dpad_right,fff))x+=2;
            if(zheng(this.gamepad1.dpad_up,ee))inchh+=1;
            if(zheng(this.gamepad1.dpad_down,ff))inchh+=1;
            if(zheng(this.gamepad1.y,m))speed+=.1;
            if(zheng(this.gamepad1.right_bumper,bF)){
                setNewGyro(0);
                int pre, cur = getXOdometry(), origin = cur;
                boolean flag = false;
                while (!flag){
                    setAllDrivePowerG(-0.5,0.5,-0.5,0.5);
                    pre = cur;
                    cur = getXOdometry();
                    if((cur-origin)/odometryEncXPerInch > inchh){
                        telemetry.addData("diff", cur-pre);
                        telemetry.update();
                        if(cur-pre < 4000){
                            flag = true;
                        }
                    }
                    Thread.sleep(100);
                }
                platform_grabber.setPower(-1);
                Thread.sleep(300);
                moveInchesGOX_platform(-16, 0.8, 1 + (13.65 - hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)) / 13.65);
                setAllDrivePower(0);
                platform_grabber.setPower(0);
                //PIDturnfast(90,false);
            }
            //telemetry.addData("s",adjustToViewMark(true)[1]);
            //telemetry.addData("s",adjustToViewMark(false)[1]);
            telemetry.addData("t", inchh);
            telemetry.update();
        }
        //cooThread.stopThread();
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
