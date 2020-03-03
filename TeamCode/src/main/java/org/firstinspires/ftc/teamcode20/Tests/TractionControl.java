package org.firstinspires.ftc.teamcode20.Tests;
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode19.Tests.WallVUMarkTesting;
import org.firstinspires.ftc.teamcode20.BaseAuto;

@Disabled
public class TractionControl extends BaseAuto {
    private double runSpeed = 0.3;
    private int delay = 200;
    private boolean DL, DR, DU, DD;

    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a){
                setAllDrivePower(0,runSpeed);
                wait(1500);
                brakeMyAss(runSpeed);
            }
            if(this.gamepad1.dpad_left){DL = true;}if(DL && !this.gamepad1.dpad_left){DL = false;
                runSpeed -= 0.01;
            }
            if(this.gamepad1.dpad_right){DR = true;}if(DR && !this.gamepad1.dpad_right){DR = false;
                runSpeed += 0.01;
            }
            if(this.gamepad1.dpad_up){DU = true;}if(DU && !this.gamepad1.dpad_up){DU = false;
                delay += 5;
            }
            if(this.gamepad1.dpad_down){DD = true;}if(DD && !this.gamepad1.dpad_down){DD = false;
                delay -= 5;
            }

            telemetry.addData("runspeed", runSpeed);
            telemetry.addData("delay", delay);
            telemetry.update();

        }
    }


    private void brakeMyAss(double speed){
        setAllDrivePower(0, -speed);
        wait(delay);
        setAllDrivePower(0);
    }
}
