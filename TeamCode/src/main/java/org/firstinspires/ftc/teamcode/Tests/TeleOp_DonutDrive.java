package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseOpMode;

@TeleOp()
public class TeleOp_DonutDrive extends BaseOpMode {

    private double outerSpeed = 0.5, innerSpeed = 0;
    @Override
    public void init() {
        initDrivetrain();
    }

    @Override
    public void loop() {
        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            outerSpeed -= 0.05;
            if(outerSpeed < 0)
                outerSpeed = 0;
        }else if(this.gamepad1.dpad_right){
            while (this.gamepad1.dpad_right);
            outerSpeed += 0.05;
            if(outerSpeed > 1)
                outerSpeed = 1;
        }
        if(this.gamepad1.dpad_down){
            while(this.gamepad1.dpad_down);
            innerSpeed -= 0.05;
            if(innerSpeed < 0)
                innerSpeed = 0;
        }else if(this.gamepad1.dpad_up){
            while(this.gamepad1.dpad_up);
            innerSpeed += 0.05;
            if(innerSpeed > 1)
                innerSpeed = 1;
        }

        if(this.gamepad1.a){
            setAllDrivePower(outerSpeed,outerSpeed,outerSpeed,innerSpeed);
        }else{
            setAllDrivePower(0);
        }

        telemetry.addLine("DPAD L/R to modify outer speed");
        telemetry.addLine("DPAD D/U to modify inner speed");
        telemetry.addLine("Hold A to run at set speeds");
        telemetry.addLine("--------------------------------");
        telemetry.addLine("Outer: "+to3d(outerSpeed)+", Inner: "+to3d(innerSpeed));
        telemetry.update();
    }
}
