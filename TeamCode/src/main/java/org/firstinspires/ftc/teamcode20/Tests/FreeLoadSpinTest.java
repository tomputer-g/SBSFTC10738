package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled

@TeleOp
public class FreeLoadSpinTest extends OpMode {
    private DcMotor motor, motor2;
    private ElapsedTime t;
    private final double encoders_per_rotation = 145.6;
    private int lastEnc;
    private double encSec;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        t = new ElapsedTime();
        motor.setPower(1);
        motor2.setPower(-1);
        lastEnc = 0;
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            motor.setPower(1);
            motor2.setPower(-1);
        }else if(this.gamepad1.b){
            motor.setPower(0);
            motor2.setPower(0);
        }else if(this.gamepad1.y){
            motor.setPower(-1);
            motor2.setPower(1);
        }
        if(this.gamepad1.x){
            motor.setPower(0);
            motor2.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(t.milliseconds() > 500){
            int diff = motor.getCurrentPosition() - lastEnc;
            lastEnc = motor.getCurrentPosition();
            encSec = 1000.0 * diff / t.milliseconds();
            t.reset();
        }
        telemetry.addData("Encoder", motor.getCurrentPosition());
        telemetry.addData("Encoders per second",encSec);
        telemetry.addData("Rotations per minute", encSec*60.0 / encoders_per_rotation);
        telemetry.update();
    }

    //L1 13.06V 1183 rpm
    //L2 13.00V 1131 rpm
    //5202: 2880 enc/s 1550 rpm
    //both 12.93V 1140
    //both other way 12.74V 1093

    //both one way 12.85 1146
    //       other 12.82 1146
}
