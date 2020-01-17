package org.firstinspires.ftc.teamcode20;

import android.provider.Telephony;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Map;
import java.util.Set;

import static java.lang.Thread.sleep;

@TeleOp(group = "Final")
public class TeleOp_MultiThreadDrive extends BaseAuto {
    private boolean BPrimed = false, RBPrimed = false, YPrimed = false, DPRPrimed = false, LPrimed = false;
    private boolean[] xprime={true},Xprimed={true};
    private boolean tapeDirectionOut = true;
    //slide
    private boolean platformGrabbed = false;


    private PWMThread pwmThread;


    @Override public void init() {
        showTelemetry = true;
        initDrivetrain();
        initGrabber();
        initLinSlide();
        initSensors();
        xOdometry = hardwareMap.get(DcMotor.class, "xOdo");
        xOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initPlatformGrabber();
        initIMU();
        setNewGyro0();
        grabber.setPosition(grabber_open);
        servoThread.setTarget(0.99);
        platform_grabber.setPower(1);
        wait(150);
        platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(0);
        pwmThread = new PWMThread();
        Set<Thread> keys = Thread.getAllStackTraces().keySet();
        Log.d("All threads log start","-------------------- "+keys.size()+"Threads -----------------------");
        for(Thread t : keys){
            Log.d("All threads: #"+t.getId(),t.getName());
        }
        Log.d("All threads log end","-------------------------------------------");
    }

    @Override public void start() {
        super.start();
        pwmThread.start();
    }

    @Override public void stop() {
        pwmThread.stopThread();
        servoThread.stopThread();
    }


    @Override
    public void loop() {

        //toggle slow
        if(this.gamepad1.y){YPrimed = true;}if(!this.gamepad1.y && YPrimed){YPrimed = false;
            if(slow==1)slow=0;
            else slow=1;
        }
        if(zheng(this.gamepad1.x,Xprimed)){
            if(slow==2)slow=0;
            else slow=2;
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
        if(this.gamepad1.right_trigger > 0.3 && (slideEncoderTravel > 0? L1.getCurrentPosition() < (slideEncoderTravel - 12 * slideEncoderPerInch) : L1.getCurrentPosition() > (slideEncoderTravel - 12 * slideEncoderPerInch)) && RTState == -1){
            //when can go 12in above & extender is extended & not started
            holdSet = false;
            autoPlaceState = -1;
            RTState = 0;
        }

        //RB toggle extender positions (not instant!)
        if(this.gamepad1.right_bumper){RBPrimed = true;}if(!this.gamepad1.right_bumper && RBPrimed){RBPrimed = false;
         if(servoThread.lastPosition > 0.75){
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
                slowModeMove(-0.3 * this.gamepad1.left_stick_x, -0.16 * this.gamepad1.left_stick_y, (this.gamepad1.left_bumper ? 0 : -0.17 * this.gamepad1.right_stick_x));
            }
        }

        //LT
        if(this.gamepad1.left_trigger  > .5 && autoPlaceState == -1){//dependent on other things?
            autoPlaceState = 0;
            RTState = -1;
        }


        //tape out/tape in
        if(this.gamepad1.dpad_left){
            LPrimed = true;
            if(tapeDirectionOut){
                xOdometry.setPower(-1);
            }else{
                xOdometry.setPower(1);
            }
        }else{
            if(LPrimed){
                LPrimed = false;
                tapeDirectionOut = !tapeDirectionOut;
            }
            xOdometry.setPower(0);
        }

        //run LT, RT, normal control
        runSlide();
        autoPlace();
        handleRTState();

        if(showTelemetry) {
            telemetry.addData("servoThread is",servoThread.getState());
            telemetry.addData("target",servoThread.targetPosition);
            telemetry.addData("actual",servoThread.lastPosition);
            telemetry.addData("RT state", RTState);
            telemetry.addData("AutoPlaceState", autoPlaceState);
            if(holdSet)telemetry.addData("Hold pos", hold);
            telemetry.addData("ext", grabber_extend1.getPosition());
            telemetry.addData("slide 1", L1.getCurrentPosition());
            telemetry.addData("tower_top dist", tower_top.getDistance(DistanceUnit.INCH) + "in.");
            telemetry.update();
        }
    }


    //-------------------------------------Multithreading---------------------------------/
    private class PWMThread extends Thread{
        volatile boolean stop = false;
        @Override
        public void run() {
            while(!isInterrupted()&&!stop){
                if(slow==1){
                    setAllDrivePowerSlow(-1*(int)gamepad1.left_stick_y,(int)(gamepad1.left_stick_x),-1*(int)(gamepad1.right_stick_x));
                    //joystickScaledMove(-0.4*gamepad1.left_stick_x,-0.13*gamepad1.left_stick_y, (gamepad1.left_bumper?0:-0.25*gamepad1.right_stick_x));
                }
            }
        }
        public void stopThread(){
            stop = true;
        }
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

}
