package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

import java.util.Set;

public class TeleOp_Demo extends BaseAuto {
    private boolean BPrimed = false, RBPrimed = false, YPrimed = false, DPRPrimed = false, LPrimed = false;
    private boolean[] xprime={true},Xprimed={true};
    private boolean tapeDirectionOut = true;
    //slide
    private boolean platformGrabbed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        showTelemetry = false;
        initGrabber();
        initLinSlide();
        initSensors();
        xOdometry = hardwareMap.get(DcMotor.class, "xOdo");
        xOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initIMU();
        setNewGyro0();
        grabber.setPosition(grabber_open);
        servoThread.setExtTarget(0.99);

        Set<Thread> keys = Thread.getAllStackTraces().keySet();
        Log.d("All threads log start","-------------------- "+keys.size()+"Threads -----------------------");
        for(Thread t : keys){
            Log.d("All threads: #"+t.getId(),t.getName());
        }
        Log.d("All threads log end","-------------------------------------------");
        waitForStart();

        while(opModeIsActive()){


            //servo toggle
            if(this.gamepad1.b && !this.gamepad1.left_bumper){BPrimed = true;}if(!this.gamepad1.b && BPrimed){BPrimed = false;
                if(grabber.getPosition() > (grabber_closed+grabber_open)/2){
                    grabber.setPosition(grabber_open);
                }else{
                    grabber.setPosition(grabber_closed);
                }
            }
            if(this.gamepad1.b && this.gamepad1.left_bumper){
                grabber.setPosition(0.01);
            }

            //driver cancel LT&RT (by dpad up/dpad down/ RB/ LB+Left stick
            if(this.gamepad1.dpad_up ||this.gamepad1.dpad_down ||this.gamepad1.right_bumper ||(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05))){
                RTState = -1; //driver interrupt auto movement
                autoPlaceState = -1;
            }

            //RT if RT not started - cancels LT
            if(this.gamepad1.right_trigger > 0.3 && RTState == -1){
                //when can go 12in above & extender is extended & not started
                holdSet = false;
                autoPlaceState = -1;
                RTState = 0;
            }

            //RB toggle extender positions (not instant!)
            if(this.gamepad1.right_bumper){RBPrimed = true;}if(!this.gamepad1.right_bumper && RBPrimed){RBPrimed = false;
                if(servoThread.extLastPosition > 0.75){
                    servoThread.setExtTarget(grabberServoOut);
                }else{
                    servoThread.setExtTarget(grabberServoIn);
                }
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
            handleRTState();

            if(showTelemetry) {
                telemetry.addData("servoThread is",servoThread.getState());
                telemetry.addData("target",servoThread.extTargetPosition);
                telemetry.addData("actual",servoThread.extLastPosition);
                telemetry.addData("RT state", RTState);
                telemetry.addData("AutoPlaceState", autoPlaceState);
                if(holdSet)telemetry.addData("Hold pos", hold);
                telemetry.addData("slide 1", L1.getCurrentPosition());
                telemetry.addData("tower_top dist", tower_top.getDistance(DistanceUnit.INCH) + "in.");
                telemetry.update();
            }
        }



        servoThread.stopThread();
    }
}
