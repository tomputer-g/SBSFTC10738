package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class MiscTest extends BaseAuto {
    //double speed,x,y, GYRO_kp, side_distance, kp,kd,dist_target,koe,square_dist;
    //boolean[] bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};
    //ElapsedTime t=new ElapsedTime();
    //ModernRoboticsI2cRangeSensor rangeSensorSide;

    //currently this is acceleration testing
    boolean a = false;
    private boolean running = false, trigger = false;
    private double impactThreshold = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initHubs();
        initIMU();
        double max = 0, current;
        Acceleration tmp;
        waitForStart();
        while(opModeIsActive()){
            //run
            if(this.gamepad1.a){a = true;}if(a && !this.gamepad1.a){
                a = false;
                running = !running;
            }
            if(running){
                setAllDrivePower(-0.5,0.5,-0.5,0.5);
            }else{
                setAllDrivePower(0);
                hub4.setLedColor(255,255,255);
            }

            //data collection
            if(this.gamepad1.b){
                max = 0;
                trigger = false;
                hub4.setLedColor(255,255,255);
            }
            tmp = imu.getLinearAcceleration();
            current = Math.sqrt(Math.pow(tmp.xAccel,2)+Math.pow(tmp.yAccel,2));
            if(current > max){
                max = current;
            }
            if(current > impactThreshold){
                trigger = true;
                hub4.setLedColor(0,255,0);
            }
            if(running)telemetry.addLine("Running");
            if(trigger)telemetry.addLine("TRIGGER");
            telemetry.addData("Current acceleration","(%2f, %2f)",tmp.xAccel,tmp.yAccel);
            telemetry.addData("Current acc. mag.", current);
            telemetry.addData("Max acc. mag.", max);
            telemetry.update();
        }

        /*
        initIMU();
        initDrivetrain();
        //rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        initPlatformGrabber();
        initSensors();
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        speed=0.55;
        y = 15;
        x = 0;
        GYRO_kp = .8;
        kd = 0;
        kp = .6;
        side_distance = 6;
        koe = 0.7;
        dist_target = 13;
        square_dist = 8.2;
        waitForStart();
        while(opModeIsActive()){
             /*
        //x+ left x- right y+ forward y- backward
        //getHeading();
        if(zheng(this.gamepad1.y,e))speed-=0.05;
        if(zheng(this.gamepad1.a,f))speed+=0.05;
        if(zheng(this.gamepad1.dpad_up,ee))square_dist+=0.5;
        if(zheng(this.gamepad1.dpad_down,ff))square_dist-=0.5;
        if(zheng(this.gamepad1.dpad_left,eee))koe+=0.02;
        if(zheng(this.gamepad1.dpad_right,fff))koe-=0.02;
        if(zheng(this.gamepad1.x,mm))side_distance++;
        if(zheng(this.gamepad1.b,mmm))side_distance--;
        telemetry.addData("speed: ","%.2f",speed);
        //telemetry.addData("x:", x);
        //telemetry.addData("y: ", y);
        //telemetry.addData("LF",LF.getCurrentPosition());
        //telemetry.addData("LB",LB.getCurrentPosition());
        //telemetry.addData("RF",RF.getCurrentPosition());
        //telemetry.addData("RB",RB.getCurrentPosition());
        //telemetry.addData("side_sensor val ", "%.2f",rangeSensorSide.getDistance(DistanceUnit.INCH));
        //telemetry.addData("side_dis", side_distance);
        telemetry.addData("square","%.2f",square_dist);
        //telemetry.addData("GYRO_kp", GYRO_kp);
        //telemetry.addData("KP", "%.1f",kp);
        telemetry.addData("koe", "%.1f",koe);
        telemetry.addData("dis", "%.1f",dist_target);
        if(zheng(this.gamepad1.back,jk)) setNewGyro0();
        if(zheng(this.gamepad1.start,bF)) {
            //moveInches(0,y,speed);
            t.reset();
            double p, dd,cur=0,pre=0, error;
            double a=-speed,b=-speed,c=speed,d=speed;
            while (t.milliseconds() < 2500) {
                cur= rangeSensorSide.getDistance(DistanceUnit.INCH) - side_distance;
                p = Math.min(0.23,Math.max(-0.23, kp *cur));
                dd = kd*(cur-pre);
                error = p+dd;
                pre = cur;
                setAllDrivePowerG(a-error,b+error,c-error,d+error);
                //sideway:
                //setAllDrivePowerG(-speed, speed, -speed, speed, GYRO_kp);
                telemetry.addData("side_sensor val ", "%.2f",rangeSensorSide.getDistance(DistanceUnit.INCH));
                telemetry.addData("Heading", imuHeading);
                telemetry.addData("t",t.milliseconds());
                telemetry.update();
            }
            //brakeTD(1);
            //setAllDrivePower(-LF.getPower()/Math.abs(LF.getPower()),-LB.getPower()/Math.abs(LB.getPower()),-RF.getPower()/Math.abs(RF.getPower()),-RB.getPower()/Math.abs(RB.getPower()));
            //wait(70);
            setAllDrivePower(0);
        }
        if(zheng(this.gamepad1.left_bumper,lF)){
            boolean flag = false ,ll = false,rr = false, far = false;
            double l,r;
            int y = 0,w = 0;
            while(!flag){


                l = left.getDistance(DistanceUnit.INCH);
                r = right.getDistance(DistanceUnit.INCH);
                telemetry.addData("l", "%.1f",l);
                telemetry.addData("r", "%.1f",r);
                telemetry.update();
                ll = near(l,square_dist,0.5);
                rr = near(r,square_dist,0.5);
                far = near(l,r,0.5);
                if (ll){
                    flag = true;
                    break;
                }
                /*
                if(far){
                    if(l>r)w=-1;
                    else w = 1;
                }
                else w = 0;



                if(!ll){
                    if(l<square_dist) y = -1;
                    else y = 1;
                }
                else y = 0;

                setAllDrivePowerSlow(y,0,w);
            }
            setAllDrivePower(0);
        }

        if (zheng(this.gamepad1.right_bumper, m)){
            /*
            platform_grabber.setPower(-.8);
            wait(300);
            turn(90, 0.67, 5);
            //while (!near(getHeading(),90,3)) setAllDrivePower(-0.6,0.2,0.8,-0.4);
            setNewGyro(90);
            //ElapsedTime p = new ElapsedTime();
            while(dist_target<rangeSensorFront.getDistance(DistanceUnit.INCH)){
                setAllDrivePowerG(koe*(0.22-0.55+0.37),koe*(0.22-0.55-0.37),koe*(0.22+0.55+0.37),koe*(0.22+0.5-0.37)); //turn+f0rwrd+side
                telemetry.addData("Front",rangeSensorFront.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
            //setAllDrivePowerG(0);
            //wait(1000);
            setAllDrivePower(0);
            while(30>rangeSensorFront.getDistance(DistanceUnit.INCH)){
                telemetry.addData("Side",rangeSensorSide.getDistance(DistanceUnit.INCH));
                telemetry.update();
                setAllDrivePowerG(0.5,-0.5,0.5,-0.5);
            }
            setAllDrivePower(0);
            platform_grabber.setPower(0);


        }
        telemetry.update();
        */

    }
}
