package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp(group = "Test")
public class The extends BaseAuto {

    private int autoPlaceState = -1;
    private int descendTarget = 0;
    private boolean BPrimed = false;

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
        autoPlace();
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
        telemetry.addData("AutoPlaceState", autoPlaceState);
        telemetry.update();
    }

    private void autoPlace(){
        switch(autoPlaceState){
            case -1:
                break;
            case 0://just started. rise to top of tower
                L1.setPower(-.3);
                L2.setPower(.3);
                telemetry.addData("Dist", tower_top.getDistance(DistanceUnit.INCH));
                if(tower_top.getDistance(DistanceUnit.INCH) > 20.0){
                    autoPlaceState++;
                    holdSet = false;
                    holdSlide(L1.getCurrentPosition() - 600);
                }
                break;
            case 1: //rise a bit more and hold position
                telemetry.addData("slide", L1.getCurrentPosition());
                telemetry.addData("holding", hold);
                if(near(hold, L1.getCurrentPosition(), 50)){
                    autoPlaceState++;
                    grabber_extender.setPower(1);
                    grabber_extender.setTargetPosition(-583);
                    grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case 2: //extend
                telemetry.addData("ext", grabber_extender.getCurrentPosition());
                if(near(grabber_extender.getCurrentPosition(), -583, 40)){
                    autoPlaceState++;
                    descendTarget = L1.getCurrentPosition() + 1100;
                    L1.setPower(0.2);
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    L2.setPower(0);
                }
                break;
            case 3: //drop & hold to correct level (descend 1200) & drop
                telemetry.addData("target", descendTarget);
                telemetry.addData("Slide pos", L1.getCurrentPosition());
                if(near(L1.getCurrentPosition(), descendTarget, 50)){
                    autoPlaceState++;
                    holdSet = false;
                    L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    holdSlide(L1.getCurrentPosition());
                    grabber.setPosition(grabber_open);
                }
                break;
            case 4: //RT - drop
                telemetry.addLine("Done, dropping");
                holdSet = false;
                RTState = 0;
                autoPlaceState = -1;


        }
    }


}
