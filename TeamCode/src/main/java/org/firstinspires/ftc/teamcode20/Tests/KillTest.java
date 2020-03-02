package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class KillTest extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        //initAutonomous();
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a){
                kill("A pressed");
            }else if(this.gamepad1.b){
                stop();
            }else{
                //LF.setPower(0.2);
                check30s();
            }
        }
    }
}
