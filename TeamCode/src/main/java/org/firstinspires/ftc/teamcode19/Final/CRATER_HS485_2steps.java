package org.firstinspires.ftc.teamcode19.Final;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.BaseTeleOp;

@TeleOp(group = "Final")
@Disabled
public class CRATER_HS485_2steps extends BaseTeleOp {
    private static final int[][] locations = {{475,0},{teleopMinDumpEncoder,0}};//shoulder joint/slide counts (skipping middle stage after servo addition)
    private static double hatch_close = 0, hatch_open = 0.4;
    private boolean autoLiftPositioning = false;


    @Override
    public void init() {
        super.init();
        grabber_shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hatch.setPosition(hatch_close);
        isMovingTo = -1;
    }

    @Override
    public void loop() {
        if(t.milliseconds() > 97000 && t.milliseconds() < 100000 && t.milliseconds() % 1000 > 500) light.setPosition(1);
        else light.setPosition(0.5);

        move(custom_linear(this.gamepad1.left_trigger > 0.5,this.gamepad1.left_stick_x), custom_linear(this.gamepad1.left_trigger > 0.5,-this.gamepad1.left_stick_y), custom_linear(this.gamepad1.left_trigger > 0.5,-this.gamepad1.right_stick_x));

        if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            if(near(hatch.getPosition(), hatch_close, 0.1)){
                hatch.setPosition(hatch_open);
            }else{
                hatch.setPosition(hatch_close);
            }
        }
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

        }else if(autoLiftPositioning){
            grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabber_slide.setPower(1);
            grabber_shoulder.setPower(1);
            grabber_slide.setTargetPosition(limitSWCalibrated ? 500 : 500 + enc_slide_offset);
            grabber_shoulder.setTargetPosition(3700);
            if(near(grabber_shoulder.getCurrentPosition(),3700, 100))
                autoLiftPositioning = false;

        }else{
            eazy_dump();
        }

        if(this.gamepad1.left_bumper && this.gamepad1.right_bumper){
            autoLiftPositioning = true;
        }else if(this.gamepad1.left_bumper){
            grabber.setPower(-0.8);
        }else if(this.gamepad1.right_bumper){
            if(this.gamepad1.left_trigger < 0.5) {
                grabber.setPower(0.35);
            }else{
                grabber.setPower(0.5);
            }
        }else{
            grabber.setPower(0);
        }

        if(grabber_shoulder.getCurrentPosition() > 3000 && grabber_shoulder.getCurrentPosition() < 4400) {
            grabber_shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else{
            grabber_shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(this.gamepad1.right_trigger>0.5){
            while(this.gamepad1.right_trigger>0.5);
            if(grabber_shoulder.getCurrentPosition()>2900) {
                isMovingTo = 0;
            }else{
                isMovingTo = 1;
            }

        }

        if(this.gamepad1.dpad_down)
            lander_lifter.setPower(-1);
        else if(this.gamepad1.dpad_up)
            lander_lifter.setPower(1);
        else
            lander_lifter.setPower(0);

        telemetry.addData("isMovingTo",isMovingTo);
        telemetry.addData("grabber shoulder",grabber_shoulder.getMode());
        telemetry.addData("lifter encoder", lander_lifter.getCurrentPosition());
        telemetry.addData("shoulder joint encoder", grabber_shoulder.getCurrentPosition());
        telemetry.addData("grabber slide encoder", (limitSWCalibrated?"limit cal. ":"")+grabber_slide.getCurrentPosition());
        telemetry.addData("hatch position", hatch.getPosition());
        telemetry.addData("t",t.milliseconds());
        if(autoLiftPositioning)telemetry.addLine("AUTO POSITIONING...");
        telemetry.update();
    }
    private void eazy_dump(){
        switch(isMovingTo){
            case -1:
                grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(grabber_shoulder.getCurrentPosition() < 3300) {
                    grabber_shoulder.setTargetPosition(grabber_shoulder.getCurrentPosition());
                    grabber_shoulder.setPower(0.5);
                }else{
                    grabber_shoulder.setPower(0);
                }
                grabber_slide.setPower(0);
                break;
            case 0://back to ground
                grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabber_shoulder.setPower(0.5);
                grabber_slide.setPower(1);
                grabber_shoulder.setTargetPosition(locations[0][0]);
                if(grabber_shoulder.getCurrentPosition() < 2500)
                    hatch.setPosition(hatch_close);
                if(grabber_shoulder.getCurrentPosition() > 2500){
                    grabber_slide.setTargetPosition(Math.max(0, (400-400*(grabber_shoulder.getCurrentPosition() - 2425)/(4100-2425))));
                }else{
                    grabber_slide.setTargetPosition(Math.max(0, 400*(grabber_shoulder.getCurrentPosition()-750)/(2425-750)));
                }
                if(near(grabber_shoulder.getCurrentPosition(),locations[0][0],70) && near(grabber_slide.getCurrentPosition(),locations[0][1],25))
                    isMovingTo = -1;
                break;
            case 1://to prepared position
                grabber_shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabber_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabber_shoulder.setPower(1);
                grabber_slide.setPower(1);
                grabber_shoulder.setTargetPosition(locations[1][0]);
                grabber_slide.setTargetPosition((limitSWCalibrated ? locations[1][1] : locations[1][1] + enc_slide_offset));
                if(near(grabber_shoulder.getCurrentPosition(),locations[1][0],70) && near(grabber_slide.getCurrentPosition(),locations[1][1],70)) {
                    isMovingTo = -1;
                    grabber_shoulder.setPower(0);
                    hatch.setPosition(hatch_open);
                }
                break;
        }

    }
}
