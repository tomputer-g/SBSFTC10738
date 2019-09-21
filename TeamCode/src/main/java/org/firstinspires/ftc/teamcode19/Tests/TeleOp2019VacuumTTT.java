package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode19.BaseAuto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
@Disabled
@TeleOp
public class TeleOp2019VacuumTTT extends BaseAuto {
    private static final double[] movement_power_params = {.1, .3, 0.8};
    private static final double ctrl_deadzone = 0.2;
    private static boolean limitSWCalibrated = false;
    private static int isMovingTo = -1; //-1 for no movement, 0/1/2 for goal locations
    private static final int[][] locations = {{0,520},{2000,300},{3500,120}};//shoulder joint/slide counts
    //red(300) w/o sorter, blue (120) w sorter
    private static final int enc_slide_offset = -750;
    private int uselight = 1;
    @Override
    public void init() {
        super.init();
        //initVuforiaEngine();
        //initVumark();
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        light.setPosition((uselight>0?1:0.5));
        move(custom_linear(this.gamepad1.left_trigger > 0.5,this.gamepad1.left_stick_x), custom_linear(this.gamepad1.left_trigger > 0.5,-this.gamepad1.left_stick_y), custom_linear(this.gamepad1.left_trigger > 0.5,-this.gamepad1.right_stick_x));

        if(this.gamepad1.a || this.gamepad1.b || this.gamepad1.x || this.gamepad1.y){//user operation: override others
                grabber_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isMovingTo = -1;
                if(this.gamepad1.x && slideLimitSW.getValue() == 0){
                    grabber_slide.setPower(-1);//extend
                }else if(this.gamepad1.b){
                    grabber_slide.setPower(1);
                }else{
                    grabber_slide.setPower(0);
                }

                if(slideLimitSW.getValue() == 1 && !near(grabber_slide.getCurrentPosition(),0,10)){
                    grabber_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    grabber_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    limitSWCalibrated = true;
                }

                grabber_shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(this.gamepad1.a){
                    grabber_shoulder.setPower(-1);

                }else if(this.gamepad1.y){
                    grabber_shoulder.setPower(1);
                }else{
                    grabber_shoulder.setPower(0);
                }
        }else if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            isMovingTo = -1;
            grabber_shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            eazy_dump();
        }/*else{
            if(isMovingShoulder && near(0,grabber_shoulder.getCurrentPosition(), 30)){
                isMovingShoulder = false;
                grabber_shoulder.setPower(0);
                grabber_shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(!isMovingShoulder){
                grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabber_shoulder.setPower(1);
                grabber_shoulder.setTargetPosition(grabber_shoulder.getCurrentPosition());
            }
            if(isMovingSlide && near((limitSWCalibrated?350:-850), grabber_slide.getCurrentPosition(), 25)) {
                isMovingSlide = false;
                grabber_slide.setPower(0);
                grabber_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(!isMovingSlide){
                grabber_slide.setPower(0);
            }
        }*/

        if(this.gamepad1.left_bumper){
            uselight = 1;
        }else if(this.gamepad1.right_bumper){
            uselight = 0;
        }else{
            grabber.setPower(0);
        }

        if(this.gamepad1.right_trigger>0.5){
            while(this.gamepad1.right_trigger>0.5);
                if(!near(grabber_shoulder.getCurrentPosition(),locations[2][0],1000)) {
                    isMovingTo = 2;
                }/*else if(grabber_shoulder.getCurrentPosition() < locations[1][0]) {
                    isMovingTo = 1;
                }else if(near(grabber_shoulder.getCurrentPosition(), locations[2][0], 200)){
                    isMovingTo = 0;
                }else{
                    isMovingTo = 1;
                }*/else{
                    isMovingTo = 0;
                }

        }

        if(this.gamepad1.dpad_down)
            lander_lifter.setPower(-1);
        else if(this.gamepad1.dpad_up)
            lander_lifter.setPower(1);
        else
            lander_lifter.setPower(0);
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }

        telemetry.update();
    }
    private void eazy_dump(){
        switch(isMovingTo){
            case -1:
                grabber_shoulder.setPower(0);
                grabber_slide.setPower(0);
                break;
            default:
                grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabber_shoulder.setPower(0.7);
                grabber_slide.setPower(1);
                grabber_shoulder.setTargetPosition(locations[isMovingTo][0]);
                grabber_slide.setTargetPosition((limitSWCalibrated ? locations[isMovingTo][1] : locations[isMovingTo][1] + enc_slide_offset));
                break;

        }
        if(isMovingTo != -1 && near(grabber_shoulder.getCurrentPosition(),locations[isMovingTo][0],100)){
            isMovingTo = -1;
        }
    }

    private double custom_quad(boolean fast, double input){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}
        double min = movement_power_params[0], max = movement_power_params[1];
        if(fast){
            min = movement_power_params[1];
            max = movement_power_params[2];
        }
        double b = min, m = max - min;
        if(input < 0)
            return -m * input * input - b;
        return m * input * input + b;
    }

    private double custom_linear(boolean fast, double input){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}
        double min = movement_power_params[0], max = movement_power_params[1];
        if(fast){
            min = movement_power_params[1];
            max = movement_power_params[2];
        }
        double b = min, m = (max - min)/(1-ctrl_deadzone);
        if(input < 0)
            return m * input - b;
        return m * input + b;
    }
}
