package org.firstinspires.ftc.teamcode20;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Thread.sleep;

@TeleOp(group = "Final")
public class TeleOp_MultiThreadDrive extends BaseAuto {
    private boolean b = false, rb = false, y = false, dpad_r = false, dpad_l = false, start = false, a = false;
    private boolean[] Xprimed={true};
    private boolean tapeDirectionOut = true;
    //slide
    private boolean platformGrabbed = false;
    Servo france;

    private int placeLevel = 0;
    private double groundHeightEnc = 2.25 * slideEncoderPerInch;
    //private PWMThread pwmThread;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime t = new ElapsedTime();
        showTelemetry = false;
        Log.i("Teleop init", ""+t.nanoseconds()+" start drivetrain");
        initDrivetrain();
        Log.i("Teleop init", ""+t.nanoseconds()+" start grabber");
        initGrabber();
        Log.i("Teleop init", ""+t.nanoseconds()+" start linSlide");
        initLinSlide();
        Log.i("Teleop init", ""+t.nanoseconds()+" start odometry");
        //initSensors();
        initOdometry();
        Log.i("Teleop init", ""+t.nanoseconds()+" start platform");
        initPlatformGrabber();
        Log.i("Teleop init", ""+t.nanoseconds()+" start IMU");
        initIMU();
        Log.i("Teleop init", ""+t.nanoseconds()+" all init done");
        setNewGyro0();
        france = hardwareMap.get(Servo.class, "capstone");
        /*2020-02-04 12:44:25.456 7969-8570/com.qualcomm.ftcrobotcontroller I/Teleop init: 92240 start drivetrain
          2020-02-04 12:44:25.639 7969-8570/com.qualcomm.ftcrobotcontroller I/Teleop init: 183704394 start grabber
          2020-02-04 12:44:25.640 7969-8570/com.qualcomm.ftcrobotcontroller I/Teleop init: 184989237 start linSlide
          2020-02-04 12:44:25.723 7969-8570/com.qualcomm.ftcrobotcontroller I/Teleop init: 267567267 start odometry
          2020-02-04 12:44:25.817 7969-8570/com.qualcomm.ftcrobotcontroller I/Teleop init: 361695922 start platform
          2020-02-04 12:44:25.818 7969-8570/com.qualcomm.ftcrobotcontroller I/Teleop init: 362624672 start IMU
          2020-02-04 12:44:27.010 7969-8570/com.qualcomm.ftcrobotcontroller I/Teleop init: 1554783072 all init done (1.5s)

         */
        servoThread.setTarget(0.99);
        grabber.setPosition(grabber_open);
        //pwmThread = new PWMThread();

        waitForStart();

        while(opModeIsActive()){
            //turn left move back onestep
            if(this.gamepad1.y&&this.gamepad1.left_bumper){
                lefty();
                slow=1;
            }

            if(this.gamepad1.y&&this.gamepad1.right_bumper){
                righty();
                slow=1;
            }

            if(this.gamepad1.y&&this.gamepad1.left_trigger>0.5){
                backy();
                slow=1;
            }

            if(zheng(this.gamepad1.x,Xprimed)){
                if(slow==2)slow=0;
                else slow=2;
            }
            //platform grabber toggle
            if(this.gamepad1.dpad_right) {
                dpad_r = true;
            }if(!this.gamepad1.dpad_right && dpad_r){
                dpad_r = false;
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


            if(this.gamepad1.a && this.gamepad1.left_trigger>0.5){
                autoPlaceLevel();
            }else if(this.gamepad1.a && this.gamepad1.right_trigger > 0.5){
                placeLevel = 0;
                autoPlaceLevel();
            }

            if(this.gamepad1.a){a = true;}if(!this.gamepad1.a && a){
                a = false;
                placeLevel++;
                autoPlaceLevel();
            }

            //servo toggle
            if(this.gamepad1.b && !this.gamepad1.left_bumper && !this.gamepad1.y){
                b = true;}if(!this.gamepad1.b && b){
                b = false;
                if(grabber.getPosition() > (grabber_closed+grabber_open)/2){
                    grabber.setPosition(grabber_open);
                }else{
                    grabber.setPosition(grabber_closed);
                }
            }
            if(this.gamepad1.b && this.gamepad1.left_bumper && !this.gamepad1.y){
                grabber.setPosition(0.01);
            }

            if(this.gamepad1.start){start = true;}if(start && !this.gamepad1.start){
                start = false;
                if(france.getPosition() >.5){
                    france.setPosition(0);
                }else{
                    france.setPosition(.5);
                }
            }

            //driver cancel LT&RT (by dpad up/dpad down/ RB/ LB+Left stick
            if(this.gamepad1.dpad_up ||this.gamepad1.dpad_down ||this.gamepad1.right_bumper ||(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05))){
                RTState = -1; //driver interrupt auto movement
                autoPlaceState = -1;
            }

            //RT if RT not started - cancels LT
            if(this.gamepad1.right_trigger > 0.3 && RTState == -1 && !this.gamepad1.y){
                //when can go 12in above & extender is extended & not started
                holdSet = false;
                autoPlaceState = -1;
                RTState = 0;
            }

            //RB toggle extender positions (not instant!)
            if(this.gamepad1.right_bumper){
                rb = true;}if(!this.gamepad1.right_bumper && rb){
                rb = false;
                if(servoThread.lastPosition > (grabberServoOut+grabberServoIn)/2){
                    servoThread.setTarget(grabberServoOut);
                }else{
                    servoThread.setTarget(grabberServoIn);
                }
            }

            //DUP/DDOWN manual extender movement
            //manual extender movement is now in servoThread.


            //If not PWM: run full speed
            if(autoPlaceState != 1) {
                if (slow==0) {//only one direction at a time
                    joystickScaledMove(-this.gamepad1.left_stick_x, -this.gamepad1.left_stick_y, (this.gamepad1.left_bumper ? 0 : -this.gamepad1.right_stick_x));
                } else if(slow==2) {
                    slowModeMove(-0.35 * this.gamepad1.left_stick_x, -0.16 * this.gamepad1.left_stick_y, (this.gamepad1.left_bumper ? 0 : -0.3 * this.gamepad1.right_stick_x));
                }else if(slow==1)
                    slowModeMove(-0.6 * this.gamepad1.left_stick_x, -0.3 * this.gamepad1.left_stick_y, (this.gamepad1.left_bumper ? 0 : -0.3 * this.gamepad1.right_stick_x));
            }

            //LT
        /*if(this.gamepad1.left_trigger  > .5 && autoPlaceState == -1){//dependent on other things?
            autoPlaceState = 0;
            RTState = -1;
        }

         */


            //tape out/tape in
            if(this.gamepad1.dpad_left){
                dpad_l = true;
                if(tapeDirectionOut){
                    xOdometry.setPower(-1);
                }else{
                    xOdometry.setPower(1);
                }
            }else{
                if(dpad_l){
                    dpad_l = false;
                    tapeDirectionOut = !tapeDirectionOut;
                }
                xOdometry.setPower(0);
            }

            //run LT, RT, normal control
            runSlide();
            //autoPlace();
            handleRTState();

            if(showTelemetry) {
                telemetry.addData("slide 1", L1.getCurrentPosition());
                telemetry.addData("slide auto level",placeLevel);
                telemetry.addData("servo actual",servoThread.lastPosition);
                telemetry.addData("RT state", RTState);
                if(holdSet)telemetry.addData("Hold pos", hold);
                telemetry.update();
            }
        }
        //pwmThread.stopThread();
        servoThread.stopThread();
    }



    private void autoPlaceLevel(){
        int goalEnc = (int)(slideEncoderPerInch * 4 * placeLevel + groundHeightEnc);//per inch is already neg.
        holdSet = false;
        holdSlide(goalEnc);
    }


    protected void joystickScaledMove(double vx, double vy, double vr){
        double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
        double absMax = 0;
        for(double d : speeds)
            absMax = Math.max(Math.abs(d),absMax);
        if(Math.abs(vx) < 0.01 && Math.abs(vy) < 0.01 && Math.abs(vr) < 0.01){
            setAllDrivePower(0);
        }else if(absMax <= 1){
            setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
        }else{
            setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }

    }

    protected void slowModeMove(double vx, double vy, double vr){
        double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
        double absMax = 0;
        for(double d : speeds)
            absMax = Math.max(Math.abs(d),absMax);
        if(absMax <= 1 && Math.abs(vr) < 0.01){
            setAllDrivePowerG(speeds[0], speeds[1], speeds[2], speeds[3]);
        }else if(Math.abs(vr) < 0.01){
            if(showTelemetry)telemetry.addLine("SCALED power: max was "+absMax);
            setAllDrivePowerG(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }else if(absMax <= 1){
            setNewGyro0();
            setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
        }else{
            setNewGyro0();
            setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }
        if(Math.abs(vx) < 0.01 && Math.abs(vy) < 0.01 && Math.abs(vr) < 0.01){
            setNewGyro0();
            setAllDrivePower(0);
        }
    }

    //-------------------------------------Multithreading---------------------------------/
    private class PWMThread extends Thread{
        volatile boolean stop = false;
        @Override
        public void run() {
            while(!isInterrupted()&&!stop){
                if(slow==1){

                    setAllDrivePowerSlow(-1*(int)gamepad1.left_stick_y,(int)(gamepad1.left_stick_x),-0.7*(int)(gamepad1.right_stick_x));
                    //joystickScaledMove(-0.4*gamepad1.left_stick_x,-0.13*gamepad1.left_stick_y, (gamepad1.left_bumper?0:-0.25*gamepad1.right_stick_x));
                }else if(slow == 0){
                    setNewGyro0();
                }
            }
        }
        public void stopThread(){
            stop = true;
        }
    }

    private class CompositeMoveThread extends Thread{
        boolean stop = false;
        @Override
        public void run() {
            while(!stop && !isInterrupted()){
                
            }
        }

        public void stopThread(){
            stop = true;
        }
    }
}
