package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Thread.sleep;

@TeleOp(group = "Final")
public class TeleOp_MultiThreadDrive extends BaseAuto {
    private boolean b = false, rb = false, y = false, dpad_r = false, dpad_l = false, start = false;
    private boolean[] Xprimed={true};
    private boolean tapeDirectionOut = true;
    //slide
    private boolean platformGrabbed = false;
    Servo france;

    //private PWMThread pwmThread;


    @Override public void init() {
        showTelemetry = false;
        initDrivetrain();
        initGrabber();
        initLinSlide();
        //initSensors();
        initOdometry();
        initPlatformGrabber();
        initIMU();
        setNewGyro0();
        france = hardwareMap.get(Servo.class, "capstone");
        servoThread.setTarget(0.99);
        grabber.setPosition(grabber_open);
        //pwmThread = new PWMThread();
    }

    @Override public void start() {
        //cooThread.start();
        //pwmThread.start();
    }


    @Override
    public void loop() {
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
            if(france.getPosition() > 0.5){
                france.setPosition(0);
            }else{
                france.setPosition(1);
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
            telemetry.addData("servoThread is",servoThread.getState());
            telemetry.addData("target",servoThread.targetPosition);
            telemetry.addData("actual",servoThread.lastPosition);
            telemetry.addData("RT state", RTState);
            telemetry.addData("AutoPlaceState", autoPlaceState);
            if(holdSet)telemetry.addData("Hold pos", hold);
            telemetry.addData("ext", grabber_extend1.getPosition());
            telemetry.addData("x", getXOdometry());
            telemetry.update();
        }
    }

    @Override public void stop() {
        //pwmThread.stopThread();
        servoThread.stopThread();
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
