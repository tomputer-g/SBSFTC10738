package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

import static java.lang.Thread.sleep;

@TeleOp(group = "Final")
public class TeleOp_MultiThreadDrive extends BaseAuto {
    private boolean BPrimed = false, RBPrimed = false, YPrimed = false, DPRPrimed = false;
    private boolean[] xprime={true};
    private boolean movingExtender = false;
    //slide
    private boolean platformGrabbed = false;


    private PWMThread pwmThread;


    @Override public void init() {
        telemetryOn = false;
        initDrivetrain();
        initGrabber();
        initLinSlide();
        initSensors();
        initPlatformGrabber();
        //initOdometry();
        initIMU();
        grabber.setPosition(grabber_open);
        grabber_extender.setPower(1);
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

    @Override public void start() {
        super.start();
        pwmThread.start();
    }

    @Override public void stop() {
        pwmThread.stopThread();
        super.stop();
    }

    @Override
    public void loop() {

        //toggle slow
        if(this.gamepad1.y){YPrimed = true;}if(!this.gamepad1.y && YPrimed){YPrimed = false;
            slow = !slow;
        }

        //platform grabber toggle
        if(this.gamepad1.dpad_right) {
            DPRPrimed = true;
        }if(!this.gamepad1.dpad_right && DPRPrimed){
            DPRPrimed = false;
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
        if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed){BPrimed = false;
            if(grabber.getPosition() > (grabber_closed+grabber_open)/2){
                grabber.setPosition(grabber_open);
            }else{
                grabber.setPosition(grabber_closed);
            }
        }

        //driver cancel LT&RT (by dpad up/dpad down/ RB/ LB+Left stick
        if(this.gamepad1.dpad_up ||this.gamepad1.dpad_down ||this.gamepad1.right_bumper ||(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05))){
            RTState = -1; //driver interrupt auto movement
            autoPlaceState = -1;
        }

        //RT if RT not started - cancels LT
        if(this.gamepad1.right_trigger > 0.3 && (slideEncoderTravel > 0? L1.getCurrentPosition() < (slideEncoderTravel - 12 * slideEncoderPerInch) : L1.getCurrentPosition() > (slideEncoderTravel - 12 * slideEncoderPerInch)) && grabber_extender.getCurrentPosition() < extenderTravel/2 && RTState == -1){
            //when can go 12in above & extender is extended & not started
            holdSet = false;
            autoPlaceState = -1;
            RTState = 0;
        }

        //RB toggle extender positions
        if(this.gamepad1.right_bumper){RBPrimed = true;}if(!this.gamepad1.right_bumper && RBPrimed){RBPrimed = false;
            movingExtender = true;
            grabber_extender.setPower(1);
            if(grabber_extender.getCurrentPosition() > extenderTravel/2){
                grabber_extender.setTargetPosition(extenderTravel);
            }else{
                grabber_extender.setTargetPosition(0);
            }
            grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //DUP/DDOWN manual extender movement
        if(this.gamepad1.dpad_up){
            movingExtender = false;
            grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            grabber_extender.setPower(-1);
        }else if(this.gamepad1.dpad_down){
            movingExtender = false;
            grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            grabber_extender.setPower(1);
        }else{
            if(RTState == -1 && autoPlaceState == -1) {
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

        //X brake (nobody cares)
        if(zheng(this.gamepad1.x,xprime)) {
            brake();//TODO: blocks thread up to 440 ms
        }

        //If not PWM: run full speed
        if(!slow){
            joystickScaledMove(-this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y, (this.gamepad1.left_bumper?0:-this.gamepad1.right_stick_x));
        }

        //LT
        if(this.gamepad1.left_trigger  > .5 && autoPlaceState == -1){//dependent on other things?
            autoPlaceState = 0;
            RTState = -1;
        }

        //run LT, RT, normal control
        runSlide();
        autoPlace();
        handleRTState();

        if(telemetryOn) {
            telemetry.addData("RT state", RTState);
            telemetry.addData("AutoPlaceState", autoPlaceState);
            if(holdSet)telemetry.addData("Hold pos", hold);
            telemetry.addData("ext", grabber_extender.getCurrentPosition());
            telemetry.addData("slide 1", L1.getCurrentPosition());
            telemetry.addData("Y", L2.getCurrentPosition());
            telemetry.addData("tower_top dist", tower_top.getDistance(DistanceUnit.INCH) + "in.");
            telemetry.update();
        }
    }


    //-----------------------------------------------------------------------------------Multithreading-------------------------------------------------------------------------
    private class PWMThread extends Thread{
        volatile boolean stop = false;
        @Override
        public void run() {
            while(!isInterrupted()&&!stop){
                if(slow){
                    setAllDrivePowerSlow(-1*(int)gamepad1.left_stick_y,(int)(gamepad1.left_stick_x),-1*(int)(gamepad1.right_stick_x));
                }
            }
        }
        public void stopThread(){
            stop = true;
        }
    }

    protected void joystickScaledMove(double vx, double vy, double vr){
        if(Math.abs(vx) > 0.2 || Math.abs(vy) > 0.2 || Math.abs(vr) > 0.2){//deadzone
            double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
            double absMax = 0;
            for(double d : speeds)
                absMax = Math.max(Math.abs(d),absMax);
            if(absMax <= 1){
                setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
            }else{
                if(telemetryOn)telemetry.addLine("SCALED power: max was "+absMax);
                setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
            }
        }
        else setAllDrivePower(0);
    }

}
