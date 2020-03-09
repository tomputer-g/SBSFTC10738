package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;

@TeleOp
public class SideAutonTest extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        servoThread.setTarget(0.99);
        grabber.setPosition(grabber_open);
        platform_grabber.setPower(1);
        platform_grabber.setPower(0.0);
        //pwmThread = new PWMThread();
        waitForStart();
        while (opModeIsActive()) {

        }
    }
    private void forleft(){
        double target=90;
        setAllDrivePower(-1,-1,1,1);
    }
}