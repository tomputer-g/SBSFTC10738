package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp(group = "Test")
public class The extends BaseAuto {
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
        runSlide();
        autoPlace();
        //handleRTState();

        if(this.gamepad1.dpad_up
                ||this.gamepad1.dpad_down
                ||this.gamepad1.right_bumper
                ||(this.gamepad1.left_bumper && !near(this.gamepad1.right_stick_y, 0, 0.05))){

            RTState = -1; //driver interrupt auto movement
        }

        if(this.gamepad1.right_trigger > 0.3
                && (slideEncoderTravel > 0? L1.getCurrentPosition() < (slideEncoderTravel - 12 * slideEncoderPerInch) : L1.getCurrentPosition() > (slideEncoderTravel - 12 * slideEncoderPerInch))
                && grabber_extender.getCurrentPosition() < -200
                && RTState == -1){
            //when can go 12in above & extender is extended & not started
            holdSet = false;
            RTState = 0;
        }
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
        telemetry.addData("RTState",RTState);
        telemetry.update();
    }

}
