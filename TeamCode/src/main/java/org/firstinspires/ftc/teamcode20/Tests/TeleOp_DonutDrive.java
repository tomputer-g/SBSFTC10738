package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class TeleOp_DonutDrive extends BaseOpMode {

    private double outerSpeed = 0.5, innerSpeed = 0, cornerSpeed = 0.5;
    @Override
    public void init() {
        initDrivetrain();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            outerSpeed -= 0.05;
            if(outerSpeed < -1)
                outerSpeed = -1;
        }else if(this.gamepad1.dpad_right){
            while (this.gamepad1.dpad_right);
            outerSpeed += 0.05;
            if(outerSpeed > 1)
                outerSpeed = 1;
        }
        if(this.gamepad1.dpad_down){
            while(this.gamepad1.dpad_down);
            innerSpeed -= 0.05;
            if(innerSpeed < -1)
                innerSpeed = -1;
        }else if(this.gamepad1.dpad_up){
            while(this.gamepad1.dpad_up);
            innerSpeed += 0.05;
            if(innerSpeed > 1)
                innerSpeed = 1;
        }
        if(this.gamepad1.x){
            while(this.gamepad1.x);
            cornerSpeed -= 0.05;
            if(cornerSpeed < -1)
                cornerSpeed = -1;
        }else if(this.gamepad1.b){
            while(this.gamepad1.b);
            cornerSpeed += 0.05;
            if(cornerSpeed > 1)
                cornerSpeed = 1;
        }

        if(this.gamepad1.a){
            setAllDrivePower(-outerSpeed,-cornerSpeed,cornerSpeed,innerSpeed);
        }else{
            setAllDrivePower(0);
        }

        telemetry.addLine("DPAD L/R to modify outer vr");
        telemetry.addLine("DPAD D/U to modify inner vr");
        telemetry.addLine("X/B to modify corner speeds");
        telemetry.addLine("Hold A to run at set speeds");
        telemetry.addLine(""+to3d(outerSpeed)+"  |  "+to3d(cornerSpeed));
        telemetry.addLine("-----------------");
        telemetry.addLine(""+to3d(cornerSpeed)+"  |  "+to3d(innerSpeed));
        telemetry.update();
    }
}
