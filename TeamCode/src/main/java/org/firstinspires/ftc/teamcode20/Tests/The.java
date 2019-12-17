package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp(group = "Test")
public class The extends BaseAuto {


    private int descendTarget = 0, ascendTarget = 0;
    private boolean BPrimed = false;
    private double inchApproachTarget = 10.0, approachSpeed = 0.2;

    @Override
    public void init() {
        initDrivetrain();
        initSensors();
        initGrabber();
        initLinSlide();
        grabber.setPosition(grabber_open);
        grabber_extender.setPower(1);
        wait(1000);
        grabber_extender.setPower(0);
        grabber_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        runSlide();
        autoPlace();
        handleRTState();


        if(this.gamepad1.b){BPrimed = true;}if(!this.gamepad1.b && BPrimed){BPrimed = false;
            if(grabber.getPosition() > (grabber_closed+grabber_open)/2){
                grabber.setPosition(grabber_open);
            }else{
                grabber.setPosition(grabber_closed);
            }
        }
        if(this.gamepad1.a && autoPlaceState == -1){
            autoPlaceState = 0;
        }
        if(holdSet)telemetry.addData("Hold", hold);
        telemetry.addData("slide", L1.getCurrentPosition());
        telemetry.addData("Power", L1.getPower());
        telemetry.addData("ext", grabber_extender.getCurrentPosition());
        telemetry.addData("Dist", tower_top.getDistance(DistanceUnit.INCH));
        telemetry.addData("AutoPlaceState", autoPlaceState);
        telemetry.addData("target", descendTarget);
        telemetry.addData("RTState",RTState);
        telemetry.update();
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
                L1.setPower(-.5);
                L2.setPower(.5);
                if(tower_top.getDistance(DistanceUnit.INCH) > 20.0 || L1.getCurrentPosition() < -5800){
                    autoPlaceState++;
                    ascendTarget = L1.getCurrentPosition() - 800;
                    L1.setPower(-.4);
                    L2.setPower(.4);
                }
                break;
            case 2: //rise a bit more and hold position
                if(ascendTarget + 50 > L1.getCurrentPosition()){
                    L1.setPower(0);
                    L2.setPower(0);
                    holdSet = false;
                    holdSlide(L1.getCurrentPosition());
                    autoPlaceState++;
                    grabber_extender.setPower(1);
                    grabber_extender.setTargetPosition(-583);
                    grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case 3: //extend

                if(near(grabber_extender.getCurrentPosition(), -583, 40)){
                    autoPlaceState++;
                    holdSet = false;
                    descendTarget = L1.getCurrentPosition() + 1300;
                    L1.setPower(0.7);
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    L2.setPower(0);
                }
                break;
            case 4: //drop & hold to correct level (descend 1200) & drop
                if(near(L1.getCurrentPosition(), descendTarget, 50)){
                    autoPlaceState++;
                    holdSet = false;
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    holdSlide(L1.getCurrentPosition());
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


}
