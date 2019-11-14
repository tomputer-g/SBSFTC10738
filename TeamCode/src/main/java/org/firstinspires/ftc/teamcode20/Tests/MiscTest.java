package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;

import static java.lang.Math.sqrt;

@TeleOp
public class MiscTest extends BaseAuto {
    double speed;
    boolean[] bF={true};

    private void 三天之内刹了你(){
        setAllDrivePower(1,1,-1,-1);
        wait(200);
        setAllDrivePower(0);
    }

    @Override
    public void init(){
        initDrivetrain();
        LF.setTargetPosition(100);
        LB.setTargetPosition(100);
        RF.setTargetPosition(100);
        RB.setTargetPosition(100);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setPower(0.05);
        LB.setPower(0.05);
        RF.setPower(0.05);
        RB.setPower(0.05);

        ElapsedTime t = new ElapsedTime();// 三天之内刹了你();
        while(t.milliseconds() < 5000){
            telemetry.addData("power", LF.getPower());
            telemetry.addData("position", LF.getCurrentPosition());
            telemetry.update();
        }
    }


    protected void moveInches(double xInch, double yInch, double speed){
        double xmult = 14./1.2, ymult = 14./1.2, p_mult = 80;
        int p_time = (int) (sqrt(xInch*xInch + yInch*yInch)*p_mult);
        ElapsedTime t = new ElapsedTime();
        int encoder_x = (int)(xInch * xmult), encoder_y = (int)(yInch * ymult);
        /*
        int encoder_1 = Math.abs(encoder_x + encoder_y); // LB, RF
        int encoder_2 = Math.abs(encoder_x - encoder_y); // LF, RB
        double conversion_fct = speed/((encoder_1 + encoder_2)/2);
        double speed_1 = conversion_fct * encoder_1, speed_2 = conversion_fct * encoder_2;
        */
        //setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        setAllDrivePower(-speed,-speed,speed,speed);
        //telemetry.addData("speed",speed_1+" "+speed_2);
        telemetry.addData("position",encoder_x+" "+encoder_y);
        telemetry.update();
        LF.setTargetPosition(encoder_x - encoder_y);
        LB.setTargetPosition(-encoder_x - encoder_y);
        RF.setTargetPosition(encoder_x + encoder_y);
        RB.setTargetPosition(-encoder_x + encoder_y);
        setMode_RESET_AND_RUN_TO_POSITION();
        while((LF.isBusy()||LB.isBusy()||RF.isBusy()||RB.isBusy()) && t.milliseconds() < p_time){};
        //setAllDrivePower(0);
        setMode_RUN_WITH_ENCODER();
        setAllDrivePower(1,1,-1,-1);
        wait(100);
        setAllDrivePower(0);
    }
}
