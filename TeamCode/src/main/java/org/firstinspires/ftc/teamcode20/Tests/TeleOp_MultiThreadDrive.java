package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

import static java.lang.Thread.sleep;

@TeleOp(group = "Final")
public class TeleOp_MultiThreadDrive extends BaseAuto {
    private boolean BPrimed = false, RBPrimed = false, YPrimed = false, XPrimed = false, LP, RP, LTPrimed = false;
    private boolean[] xprime={true};
    private boolean movingExtender = false;
    //slide
    private double kickstartSpeed = 0.18, lowSpeed = 0.03, PWMSpeed = 0.09, cycleTimeMS = 20;
    private boolean platformGrabbed = false;

    private int descendTarget = 0, ascendTarget = 0;
    private double inchApproachTarget = 10.0, approachSpeed = 0.2;
    private PWMThread pwmThread;


    @Override public void init() {
        telemetryOn = true;
        initDrivetrain();
        initGrabber();
        initLinSlide();
        initSensors();
        initPlatformGrabber();
        initIMU();
        grabber.setPosition(grabber_open);
        grabber_extender.setPower(1);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(1);
        wait(150);
        platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(0);
        wait(500);
        grabber_extender.setPower(0);
        grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pwmThread = new PWMThread();
    }

    @Override
    public void start() {
        super.start();
        pwmThread.start();
    }

    @Override
    public void stop() {
        pwmThread.stopThread();
        super.stop();
    }

    @Override
    public void loop() {
        if(telemetryOn)telemetry.addData("RT state", RTState);
        runSlide();
        autoPlace();
        if(holdSet){if(telemetryOn)telemetry.addData("Hold pos", hold);}

        if(this.gamepad1.y){YPrimed = true;}if(!this.gamepad1.y && YPrimed){YPrimed = false;
            slow = !slow;
        }


        if(this.gamepad1.left_trigger> 0.5) {
            LTPrimed = true;
        }
        if(this.gamepad1.left_trigger < 0.5 && LTPrimed){
            LTPrimed = false;
            if(platformGrabbed){//already held
                platformGrabbed = false;
                platform_grabber.setPower(1);
                wait(100);
                platform_grabber.setPower(0);
            }else{
                platformGrabbed = true;
                platform_grabber.setPower(-0.4);
            }
        }


        if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed){BPrimed = false;
            if(grabber.getPosition() > (grabber_closed+grabber_open)/2){
                grabber.setPosition(grabber_open);
            }else{
                grabber.setPosition(grabber_closed);
            }
        }

        /*
        if(this.gamepad1.x){XPrimed = true;}if(!this.gamepad1.x && XPrimed){XPrimed = false;
            setNewGyro(0);
            while ( !(near(left.getDistance(DistanceUnit.INCH),9.027, 0.6)&&near(right.getDistance(DistanceUnit.INCH),8.125, 0.6)) ){
                double l = left.getDistance(DistanceUnit.INCH), r = right.getDistance(DistanceUnit.INCH);
                l = Math.min(1,Math.abs(l-9.027));
                if     (r > 9.027)setAllDrivePowerG(-0.13*l,-0.13*l,0.13*l,0.13*l);
                else if(r < 9.027)setAllDrivePowerG(0.13*l,0.13*l,-0.13*l,-0.13*l);
            }
            setAllDrivePower(0);
        }

         */

        if(this.gamepad1.dpad_up
                ||this.gamepad1.dpad_down
                ||this.gamepad1.right_bumper
                ||(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05))){

            RTState = -1; //driver interrupt auto movement
        }

        if(this.gamepad1.right_trigger > 0.3
                && L1.getCurrentPosition() > (-6500 + 12 * encoderPerInch)
                && grabber_extender.getCurrentPosition() < -200
                && RTState == -1){
            //when can go 12in above & extender is extended & not started
            holdSet = false;
            RTState = 0;
        }
        handleRTState();

        if(this.gamepad1.right_bumper){RBPrimed = true;}if(!this.gamepad1.right_bumper && RBPrimed){RBPrimed = false;
            movingExtender = true;
            grabber_extender.setPower(1);
            if(grabber_extender.getCurrentPosition() > -200){
                grabber_extender.setTargetPosition(-583);
            }else{
                grabber_extender.setTargetPosition(0);
            }
            grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }//if RB toggle positions


        if(this.gamepad1.dpad_up){
            movingExtender = false;
            grabber_extender.setPower(-1);
        }else if(this.gamepad1.dpad_down){
            movingExtender = false;
            grabber_extender.setPower(1);
        }else{
            if(RTState == -1) {
                if (!movingExtender) {
                    grabber_extender.setPower(0);
                } else {
                    if (!grabber_extender.isBusy()) {
                        movingExtender = false;
                        grabber_extender.setPower(0);
                        grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }
        }

        if(this.gamepad1.a){
            grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(zheng(this.gamepad1.x,xprime)) {
            brake();
        }

        if(L1.getCurrentPosition() < -200){
            if(telemetryOn)telemetry.addLine("Placing block: ultra slow");
            /*if(-this.gamepad1.left_stick_y > ctrl_deadzone){
                try {
                    movePWM(0.09);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }else if(this.gamepad1.left_stick_y > ctrl_deadzone){
                try {
                    movePWM(-0.09);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

             */
            scaledMove(-this.gamepad1.left_stick_x*0.3,-this.gamepad1.left_stick_y*0.15, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x*0.2));//replace with enc-based move
        }else if(!slow)
            //below is slow code
            //if(telemetryOn)telemetry.addLine("slide and drivetrain slowed");
            //scaledMove(-this.gamepad1.left_stick_x*0.4,-this.gamepad1.left_stick_y*0.4, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x*0.3));
            //setAllDrivePowerSlow(-1*(int)this.gamepad1.left_stick_y,(int)(this.gamepad1.left_stick_x),-1*(int)(this.gamepad1.right_stick_x));
        {
            scaledMove(-this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x));
        }

        if(this.gamepad1.left_trigger  > .5 && autoPlaceState == -1){
            autoPlaceState = 0;
        }
        telemetry.addData("AutoPlaceState", autoPlaceState);
        telemetry.addData("target", descendTarget);

        //if(telemetryOn)telemetry.addData("a",a);
        //if(telemetryOn)telemetry.addLine("Dist: "+left.getDistance(DistanceUnit.INCH)+", "+right.getDistance(DistanceUnit.INCH));
        telemetry.addData("ext", grabber_extender.getCurrentPosition());
        telemetry.addData("slide 1",L1.getCurrentPosition());

        telemetry.addData("tower_top dist", tower_top.getDistance(DistanceUnit.INCH)+"in.");
        telemetry.update();
    }


    //-----------------------------------------------------------------------------------Multithreading-------------------------------------------------------------------------
    private class PWMThread extends Thread{
        volatile boolean stop = false;
        @Override
        public void run() {
            while(!isInterrupted()&&!stop){
                if(slow && L1.getCurrentPosition() < -200){
                    telemetry.addLine("Thread running");
                    setAllDrivePowerSlow(-1*(int)gamepad1.left_stick_y,(int)(gamepad1.left_stick_x),-1*(int)(gamepad1.right_stick_x));
                }
            }
        }
        public void stopThread(){
            stop = true;
        }
    }


    private class AutoPlaceBlockThread extends Thread{
        @Override
        public void run(){
            holdSlide((int) (L1.getCurrentPosition() - 12 * encoderPerInch));
            grabber.setPosition(0);
            while (near(hold, L1.getCurrentPosition(), 100));//close enough

        }
    }


    private void handleRTState(){//call in loop; non-blocking
        switch (RTState) {
            case -1: //none
                break;
            case 0: //just pressed button / moving upward 12 in
                holdSlide((int) (L1.getCurrentPosition() - 12 * encoderPerInch));
                grabber.setPosition(0);
                if (near(hold, L1.getCurrentPosition(), 100))//close enough
                    RTState = 1;
                break;
            case 1:
                grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                grabber_extender.setPower(1);
                if (near(grabber_extender.getCurrentPosition(), 0, 20)){
                    grabber_extender.setPower(0);
                    RTState = 2;
                }
                break;
            case 2://need -.5 power going down, test this
                holdSet = false;
                L1.setPower(0.8);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                L2.setPower(0);
                if(L1.getCurrentPosition() > -40){
                    RTState = -1;
                    L1.setPower(0);
                    L2.setPower(0);
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                break;
        }
    }

    private void autoPlace(){
        switch(autoPlaceState){
            case -1:
                break;
            case 0://approach
                if(tower_top.getDistance(DistanceUnit.INCH) > inchApproachTarget + 0.5){
                    setAllDrivePower(-approachSpeed,-approachSpeed,approachSpeed,approachSpeed);
                }else if(tower_top.getDistance(DistanceUnit.INCH) < inchApproachTarget - 0.5){
                    setAllDrivePower(approachSpeed, approachSpeed, -approachSpeed, -approachSpeed);
                }else{
                    autoPlaceState++;
                    setAllDrivePower(0);
                }
                break;
            case 1://just started. rise to top of tower
                setAllDrivePower(0);
                L1.setPower(-1);
                L2.setPower(1);
                if(tower_top.getDistance(DistanceUnit.INCH) > 20.0 || L1.getCurrentPosition() < -5800){
                    autoPlaceState++;
                    ascendTarget = L1.getCurrentPosition() - 800;
                    L1.setPower(-.7);
                    L2.setPower(.7);
                    grabber_extender.setPower(1);
                    grabber_extender.setTargetPosition(-583);
                    grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case 2: //rise a bit more and hold position
                if(ascendTarget + 50 > L1.getCurrentPosition()){
                    L1.setPower(0);
                    L2.setPower(0);
                    holdSet = false;
                    holdSlide(L1.getCurrentPosition());
                    autoPlaceState++;
                }
                break;
            case 3: //extend

                if(near(grabber_extender.getCurrentPosition(), -583, 40)){
                    autoPlaceState++;
                    holdSet = false;
                    descendTarget = L1.getCurrentPosition() + 1350;
                    L1.setPower(0.7);
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    L2.setPower(0);
                }
                break;
            case 4: //drop & hold to correct level (descend 1200) & drop
                if(near(L1.getCurrentPosition(), descendTarget, 50)){
                    //autoPlaceState++;
                    holdSet = false;
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    holdSlide(L1.getCurrentPosition());
                    autoPlaceState = -1;
                    //grabber.setPosition(grabber_open);
                }
                break;
            /*case 5: //RT - drop
                holdSet = false;
                RTState = 0;
                autoPlaceState = -1;

             */
        }
    }

}
