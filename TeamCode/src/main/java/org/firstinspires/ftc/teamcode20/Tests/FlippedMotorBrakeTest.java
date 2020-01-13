package org.firstinspires.ftc.teamcode20.Tests;


import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp
public class FlippedMotorBrakeTest extends OpMode {
    private DcMotor LF, LB, RF, RB;

    @Override
    public void init() {
        LF = hardwareMap.get(DcMotor.class,"LF");
        LB = hardwareMap.get(DcMotor.class,"LB");
        RF = hardwareMap.get(DcMotor.class,"RF");
        RB = hardwareMap.get(DcMotor.class,"RB");
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        LB.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        RF.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        RB.setMotorType(MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){
            setAllDrivePower(0,0,0,0);
        }else if(this.gamepad1.b){
            setAllDrivePower(0.13,0.13,-0.13,-0.13);
        }else{
            joystickScaledMove(this.gamepad1.left_stick_x,this.gamepad1.left_stick_y, (this.gamepad1.left_bumper?0:this.gamepad1.right_stick_x));
        }

    }

    protected void joystickScaledMove(double vx, double vy, double vr){
        telemetry.addData("x",vx);
        telemetry.addData("y",vy);
        telemetry.update();
            double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
            double absMax = 0;
            for(double d : speeds)
                absMax = Math.max(Math.abs(d),absMax);
            if(absMax <= 1){
                setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
            }else{
                setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
            }

    }

    protected void setAllDrivePower(double pLF, double pLB, double pRF, double pRB){
        LF.setPower(-pLF);
        LB.setPower(-pLB);
        RF.setPower(-pRF);
        RB.setPower(-pRB);
    }
}
