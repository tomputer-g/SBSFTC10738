package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by Ziming Gao on 1/16/2018.
 */
@TeleOp
@Disabled
public class DoubleServoTest extends OpMode {
    private Servo servoL, servoR;
    private double lmin = 0, lmax = 0.25, rmin = 0.75, rmax = 1;
    private boolean flipDirL, flipDirR;
    private ServoController servoCtrl;
    @Override
    public void init() {
        msStuckDetectLoop=100000;
        servoL = hardwareMap.get(Servo.class,"lservo");
        servoR = hardwareMap.get(Servo.class,"rservo");
        telemetry.addLine("Ready");
        telemetry.update();
        servoCtrl = servoL.getController();
    }

    @Override
    public void loop() {
        /*
        if(this.gamepad1.a){
            while(this.gamepad1.a);
            servoCtrl.pwmDisable();
            telemetry.addLine("Setting servo limits...");
            telemetry.addLine("Press A at limits of each servo");
            telemetry.update();
            while(!this.gamepad1.a);
            while(this.gamepad1.a);
            servoCtrl.pwmEnable();
            double tempL1 = servoCtrl.getServoPosition(1);
            double tempR1 = servoCtrl.getServoPosition(2);
            servoCtrl.pwmDisable();
            telemetry.addLine("Setting servo limits...");
            telemetry.addData("L servo", tempL1);
            telemetry.addData("R servo", tempR1);
            telemetry.addLine("Push servos to other end and press A");
            telemetry.update();
            while(!this.gamepad1.a);
            while(this.gamepad1.a);
            servoCtrl.pwmEnable();
            double tempL2 = servoCtrl.getServoPosition(1);
            double tempR2 = servoCtrl.getServoPosition(2);
            servoCtrl.pwmDisable();
            lmin = tempL1;
            lmax = tempL2;
            if(tempL1 > tempL2){
                lmin = tempL2;
                lmax = tempL1;
            }
            rmin = tempR1;
            rmax = tempR2;
            if(tempR1 > tempR2){
                rmin = tempR2;
                rmax = tempR1;
            }
            telemetry.addLine("Done setting servo limits.");
            telemetry.addLine("L servo: ["+lmin+", "+lmax+"]");
            telemetry.addLine("R servo: ["+rmin+", "+rmax+"]");
            telemetry.addLine("Press A again to recalibrate");
            telemetry.update();
            servoCtrl.pwmEnable();
        }
        */
        double inputL = (this.gamepad1.left_stick_x + 1)/2;
        double inputR = (this.gamepad1.right_stick_x + 1)/2;
        if(flipDirL){
            inputL = 1-inputL;
        }
        if(flipDirR){
            inputR = 1-inputR;
        }

        double mL = lmax - lmin;
        double mR = rmax - rmin;
        servoL.setPosition(lmin+mL*inputL);
        servoR.setPosition(rmin+mR*inputR);
        telemetry.addData("L servo",servoL.getPosition());
        telemetry.addData("R servo",servoR.getPosition());
        telemetry.addLine("L range: ["+lmin+", "+lmax+"]");
        telemetry.addLine("R range: ["+rmin+", "+rmax+"]");
        telemetry.update();

    }
}


