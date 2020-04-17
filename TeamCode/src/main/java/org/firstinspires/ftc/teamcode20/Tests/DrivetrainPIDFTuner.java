package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.Roadrunner.drive.GoBildaMotor1150;
import org.openftc.revextensions2.ExpansionHubEx;

@Autonomous
public class DrivetrainPIDFTuner extends BaseAuto {
    DcMotorEx LF, LB, RF, RB;
    double currentVelocity;
    double currentPower = 0.5;
    boolean lb, rb, run = false, a;
    double maxVelocity =0;

    @Override public void runOpMode() {
        initLogger("MotorVelocityPIDF_"+System.currentTimeMillis()+".csv");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        LB.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        RF.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        RB.setMotorType(MotorConfigurationType.getMotorType(GoBildaMotor1150.class));
        ExpansionHubEx hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        double currentV = hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        double maxVel = (14/currentV) * 2152;
        double F = 32767/maxVel, P = F/10, I = P/10, D = 0;
        LF.setVelocityPIDFCoefficients(P, I, D, F);
        LB.setVelocityPIDFCoefficients(P, I, D, F);
        RF.setVelocityPIDFCoefficients(P, I, D, F);
        RB.setVelocityPIDFCoefficients(P, I, D, F);
        writeLogHeader("batt="+currentV+",maxVel="+maxVel+",PIDF="+P+" "+I+" "+D+" "+F);
        waitForStart();
        writeLogHeader("setpower,encVelocity");

        while(opModeIsActive()){
            if(this.gamepad1.a){
                a = true;
            }
            if(!this.gamepad1.a && a){
                a = false;
                run = !run;
            }
            //set powers
            if(run) {
                currentVelocity = (-LF.getVelocity() - LB.getVelocity() + RF.getVelocity() + RB.getVelocity()) / 4.0;
                LF.setPower(-currentPower);
                LB.setPower(-currentPower);
                RF.setPower(currentPower);
                RB.setPower(currentPower);
                writeLog(currentPower+","+currentVelocity);
                if(currentVelocity>maxVelocity)
                    maxVelocity=currentVelocity;
            }else{
                LF.setPower(0);
                LB.setPower(0);
                RB.setPower(0);
                RF.setPower(0);
            }

            //change power
            if(this.gamepad1.left_bumper){
                lb = true;
            }
            if(!this.gamepad1.left_bumper && lb){
                lb = false;
                currentPower -= 0.05;
                if (currentPower < 0.0)currentPower = 0;
                currentPower = Math.round(currentPower * 100) / 100.0;
            }

            if(this.gamepad1.right_bumper){
                rb = true;
            }
            if(!this.gamepad1.right_bumper && rb){
                rb = false;
                currentPower += 0.05;
                if(currentPower > 1.0)currentPower = 1;
                currentPower = Math.round(currentPower * 100) / 100.0;
            }


            telemetry.addData("current power",currentPower);
            telemetry.addData("current velocity", currentVelocity);//1215|1220|1200 on mat (with 2600 max)
            telemetry.addData("max velocity", maxVelocity);
            telemetry.update();
        }
        stopLog();

        /*
        LF.setPower(-currentPower);
        LB.setPower(-currentPower);
        RF.setPower(currentPower);
        RB.setPower(currentPower);
        while (opModeIsActive()) {
            if(!end) {
                if (this.gamepad1.left_bumper) {
                    lb = true;
                }
                if (lb && !this.gamepad1.left_bumper) {
                    lb = false;
                    currentPower -= 0.1;
                    currentPower = Math.max(currentPower, 0);
                    LF.setPower(-currentPower);
                    LB.setPower(-currentPower);
                    RF.setPower(currentPower);
                    RB.setPower(currentPower);
                }
                if (this.gamepad1.right_bumper) {
                    rb = true;
                }
                if (rb && !this.gamepad1.right_bumper) {
                    rb = false;
                    currentPower += 0.1;
                    currentPower = Math.min(currentPower, 1);
                    LF.setPower(-currentPower);
                    LB.setPower(-currentPower);
                    RF.setPower(currentPower);
                    RB.setPower(currentPower);
                }
                currentVelocity = (-LF.getVelocity() - LB.getVelocity() + RF.getVelocity() + RB.getVelocity()) / 4.0;

                if (currentVelocity > maxVelocity) {
                    maxVelocity = currentVelocity;
                }
                if (this.gamepad1.b) {
                    maxVelocity = 0;
                }
                if(this.gamepad1.a){
                    end = true;
                }
                telemetry.addData("current power", currentPower);
                telemetry.addData("current velocity", currentVelocity);
                telemetry.addData("maximum avg velocity", maxVelocity);//1170 for 0.5
                telemetry.update();
            }else{
                LF.setPower(0);
                LB.setPower(0);
                RF.setPower(0);
                RB.setPower(0);
                telemetry.addData("current power", currentPower);
                telemetry.addData("current velocity", currentVelocity);
                telemetry.addData("maximum avg velocity", maxVelocity);//1830 based on 5 trials (12.7V) -> F 17.906, P 1.791, I 0.179, D 0
                                                                               //2080 based on 2 trials (14..V) -> F 15.753, P 1.575, I 0.158, D 0
                telemetry.update();
            }
        }

         */
    }
}
