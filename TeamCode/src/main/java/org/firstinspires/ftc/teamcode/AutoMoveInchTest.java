package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode19.BaseAuto;
import org.firstinspires.ftc.teamcode19.BaseOpMode;

@TeleOp
public class AutoMoveInchTest extends BaseOpMode {
    ElapsedTime t;

    @Override public void init() {
        initDrivetrain();
        t = new ElapsedTime();
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            t.reset();
            moveInches(30,0,1);
            telemetry.addData("Time",t.milliseconds()+"ms");
            telemetry.update();
            //moveInchesHighSpeedEncoder(30,0,1,5,5,0.2,0.2,0);
        }else if(this.gamepad1.y){
            while(this.gamepad1.y);
            t.reset();
            moveInches(0,30,1);
            telemetry.addData("Time",t.milliseconds()+"ms");
            telemetry.update();
            //moveInchesHighSpeedEncoder(0,30,1,5,5,0.2,0.2,0);
        }else if(this.gamepad1.x){
            while(this.gamepad1.x);
            t.reset();
            moveInches(-30,0,1);
            telemetry.addData("Time",t.milliseconds()+"ms");
            telemetry.update();
            //moveInchesHighSpeedEncoder(-30,0,1,5,5,0.2,0.2,0);
        }else if(this.gamepad1.b) {
            while (this.gamepad1.b);
            t.reset();
            moveInches(0, -30, 1);
            telemetry.addData("Time",t.milliseconds()+"ms");
            telemetry.update();
            //moveInchesHighSpeedEncoder(0,-30,1,5,5,0.2,0.2,0);
        }
        fucc
    }
}
