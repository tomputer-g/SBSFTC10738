package org.firstinspires.ftc.teamcode18.Final;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode18.BaseOpMode;
@Disabled
@TeleOp(name = "1PControl", group = "Final")
public class TeleOpFinal extends BaseOpMode {
    private double speed = 0.5, slidePower = 0;
    private boolean grabbed = false, resetGrabber = false;
    @Override
    public void init() {
        super.init();
        grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CSArm.setPosition(0.26);
    }

    @Override
    public void loop() {
        super.loop();
        if(this.gamepad1.left_trigger > 0.5){
            speed = 0.1;
        }else if(this.gamepad1.right_trigger > 0.5){
            speed = 1.0;
        }else{
            speed = 0.5;
        }//speed ctrl (LB / RB)
        move(this.gamepad1.left_stick_x, -this.gamepad1.left_stick_y, this.gamepad1.right_stick_x);//movement(L stick, R stick)
        // Slide
        if(this.gamepad1.dpad_up){
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (slidePower < 0.65)
                slidePower += 0.02;
            slide.setPower(slidePower);
        }else if(this.gamepad1.dpad_down && slide.getCurrentPosition() > 250){
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(slidePower > -0.65)
                slidePower -= 0.02;
            slide.setPower(slidePower);
        }
        if(!this.gamepad1.dpad_up && !this.gamepad1.dpad_down){
            if(slidePower > 0)
                slidePower -= 0.035;
            if(slidePower < 0)
                slidePower += 0.035;
            if(near(slidePower,0,0.07))
                slidePower = 0;
            slide.setPower(slidePower);
        }//manual hold-to-move (dpad up, dpad down)

        if(this.gamepad1.dpad_left){//why?
            while(this.gamepad1.dpad_left);
            grabber.setTargetPosition(grabber.getCurrentPosition() - 10);
            grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            grabber.setTargetPosition(grabber.getCurrentPosition() + 10);
            grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }//manual grabber control(dpad left, dpad right)
        if(this.gamepad1.b){
            while(this.gamepad1.b);
            homeSlide();
        }//auto slide zero (B)

        if(this.gamepad1.right_bumper){
            while(this.gamepad1.right_bumper);
            grabbed = !grabbed;
            grabber.setTargetPosition(grabbed ? 115 : 0);
        }//toggle grabber (RB)
        telemetry.addData("grabber", grabber.getCurrentPosition());
        telemetry.addData("ODS",homeSlide.getVoltage());
        telemetry.addData("slide position", slide.getCurrentPosition());
        telemetry.addData("slide power", slidePower);
        telemetry.addData("angle", getAngle());
        telemetry.update();
    }
    private void move(double vx, double vy, double vr){
            LF.setPower(0.5 * speed * (vx - vy + vr));
            LB.setPower(0.5 * speed * (-vy - vx + vr));
            RF.setPower(0.5 * speed * (vx + vy + vr));
            RB.setPower(0.5 * speed * (-vx + vy + vr));
    }
}
