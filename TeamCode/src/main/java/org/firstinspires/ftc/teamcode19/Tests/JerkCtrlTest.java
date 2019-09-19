package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode19.BaseOpMode;

@TeleOp
@Disabled
public class JerkCtrlTest extends BaseOpMode {
    private static double max_0_to_1_time = 500; //ms
    private static int ctrl_mode = 0;
    private static double max_delta_v = 6 / max_0_to_1_time; //per cycle
    private static double[] lastMotorPowers = {0,0,0,0}; //LF, LB, RF, RB
    private static final double[] x_sqr_movement_params = {0.05, 0.2, 0.2, 1};
    private static final double ctrl_deadzone = 0.2;
    
    //Winston jerk control params
    private static double n = 2, T = 400;
    private static ElapsedTime t = new ElapsedTime(), t1 = new ElapsedTime();
    private static int[] lastEncoderCounts = {0,0,0,0}, encoderDiff=  new int[4];
    @Override
    public void init() {
        super.init();
        setMode_RUN_WITH_ENCODER();
    }

    @Override
    public void loop() {
        double ms = t1.milliseconds();
        if (ms >= 125) {//logging encoder speed
            t1.reset();
            encoderDiff[0] = LF.getCurrentPosition() - lastEncoderCounts[0];
            encoderDiff[1] = LB.getCurrentPosition() - lastEncoderCounts[1];
            encoderDiff[2] = RF.getCurrentPosition() - lastEncoderCounts[2];
            encoderDiff[3] = RB.getCurrentPosition() - lastEncoderCounts[3];
            for(int i = 0;i < 4; i++){
                encoderDiff[i] *= (1000/ms);
            }
        }
        telemetry.addData("Encoder counts/s: ", "["+encoderDiff[0]+", "+encoderDiff[1]+", "+encoderDiff[2]+", "+encoderDiff[3]+"]");
        if(this.gamepad1.a){//switch modes
            while(this.gamepad1.a);
            ctrl_mode++;
            if(ctrl_mode > 2){
                ctrl_mode = 0;
            }
            if(ctrl_mode == 2){
                t.reset();
            }
        }
        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            if(ctrl_mode == 1){
                max_0_to_1_time -= 10;
            }else if(ctrl_mode == 2){
                T -= 10;
            }
        }else if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            if(ctrl_mode == 1){
                max_0_to_1_time += 10;
            }else if(ctrl_mode == 2){
                T += 10;
            }
        }else if(this.gamepad1.dpad_up){
            while(this.gamepad1.dpad_up);
            if(ctrl_mode == 2){
                n += 0.025;
            }
        }else if(this.gamepad1.dpad_down){
            while(this.gamepad1.dpad_down);
            if(ctrl_mode == 2){
                n -= 0.025;
            }
        }
        switch(ctrl_mode){
            case 0:
                move(lin(this.gamepad1.left_bumper, this.gamepad1.left_stick_x), lin(this.gamepad1.left_bumper, -this.gamepad1.left_stick_y), lin(this.gamepad1.left_bumper, this.gamepad1.right_stick_x));
                break;
            case 1:
                jerk_ctrl(lin(this.gamepad1.left_bumper, this.gamepad1.left_stick_x), lin(this.gamepad1.left_bumper, -this.gamepad1.left_stick_y), lin(this.gamepad1.left_bumper, this.gamepad1.right_stick_x));
                break;
            case 2:
                jerk_ctrl_winston(lin(this.gamepad1.left_bumper, this.gamepad1.left_stick_x), lin(this.gamepad1.left_bumper, -this.gamepad1.left_stick_y), lin(this.gamepad1.left_bumper, this.gamepad1.right_stick_x));
                break;
        }
        telemetry.update();
    }

    protected void move(double vx, double vy, double vr){
        lastMotorPowers[0] = 0.5 * (vx - vy + vr);
        lastMotorPowers[1] = 0.5 * (-vy - vx + vr);
        lastMotorPowers[2] = 0.5 * (vx + vy + vr);
        lastMotorPowers[3] = 0.5 * (-vx + vy + vr);
        LF.setPower(lastMotorPowers[0]);
        LB.setPower(lastMotorPowers[1]);
        RF.setPower(lastMotorPowers[2]);
        RB.setPower(lastMotorPowers[3]);
        telemetry.addLine("Jerk control OFF");
    }

    private void jerk_ctrl(double vx, double vy, double vr){
        double[] newMotorPowers = {0.5 * (vx - vy + vr),0.5 * (-vy - vx + vr),0.5 * (vx + vy + vr),0.5 * (-vx + vy + vr)}, pwrDiff = new double[4];
        boolean jerkLimiting = false;
        double minJerk = 0, maxJerk = 0, jerkMultFactor = 1;
        for(int i = 0;i < 4; i++){
            pwrDiff[i] = newMotorPowers[i] - lastMotorPowers[i];
            if(pwrDiff[i] < 0){
                minJerk = Math.min(minJerk, pwrDiff[i]);
            }else{
                maxJerk = Math.max(maxJerk, pwrDiff[i]);
            }
        }
        if(-minJerk > max_delta_v || maxJerk > max_delta_v){
            jerkLimiting = true;
            if(-minJerk > maxJerk){
                jerkMultFactor = max_delta_v / -minJerk;
            }else{
                jerkMultFactor = max_delta_v / maxJerk;
            }
        }
        for(int i = 0;i < 4;i++){
            newMotorPowers[i] = lastMotorPowers[i] + (jerkMultFactor * pwrDiff[i]);
        }
        LF.setPower(lastMotorPowers[0]);
        LB.setPower(lastMotorPowers[1]);
        RF.setPower(lastMotorPowers[2]);
        RB.setPower(lastMotorPowers[3]);
        telemetry.addLine("Jerk control (Tom) "+ (jerkLimiting ? "ACTIVE" : "INACTIVE"));
        telemetry.addLine("DPAD left: 0 to 1 time -- | DPAD right: 0 to 1 time ++");
        telemetry.addData("0 to 1 time", max_0_to_1_time + "ms");
    }

    private void jerk_ctrl_winston(double vx, double vy, double vr){
        double[] vfs = {0.5 * (vx - vy + vr), 0.5 * (-vy - vx + vr), 0.5 * (vx + vy + vr), 0.5 * (-vx + vy + vr)};
        //for(i = v0;i < vf; i+=0.01){power += 0.01*(vf-v0)/(Math.abs(vf-v0))}
        while(t.milliseconds() < T/2) {
            double ms = t.milliseconds();
            LF.setPower(winston_eqn(lastMotorPowers[0], vfs[0], ms));
            LB.setPower(winston_eqn(lastMotorPowers[1], vfs[1], ms));
            RF.setPower(winston_eqn(lastMotorPowers[2], vfs[2], ms));
            RB.setPower(winston_eqn(lastMotorPowers[3], vfs[3], ms));
            wait(5);
        }
        while(t.milliseconds() < T) {
            double ms = t.milliseconds();
            LF.setPower(lastMotorPowers[0]/2+vfs[0]/2 - winston_eqn(lastMotorPowers[0], vfs[0], T-ms));
            LB.setPower(lastMotorPowers[1]/2+vfs[1]/2 - winston_eqn(lastMotorPowers[1], vfs[1], T-ms));
            RF.setPower(lastMotorPowers[2]/2+vfs[2]/2 - winston_eqn(lastMotorPowers[2], vfs[2], T-ms));
            RB.setPower(lastMotorPowers[3]/2+vfs[3]/2 - winston_eqn(lastMotorPowers[3], vfs[3], T-ms));
            wait(5);
        }
        t.reset();
        lastMotorPowers[0] = LF.getPower();
        lastMotorPowers[1] = LB.getPower();
        lastMotorPowers[2] = RF.getPower();
        lastMotorPowers[3] = RB.getPower();
        telemetry.addLine("Jerk control (Winston) ACTIVE");
        telemetry.addLine("DPAD left: T-- | DPAD right: T++ | DPAD down: n-- | DPAD up: n++");
        telemetry.addData("n", n);
        telemetry.addData("T", T);
    }

    //Winston 2nd:
    //for(i = v0;i < vf; i+=0.01){power += 0.01*(vf-v0)/(Math.abs(vf-v0))}

    private double winston_eqn(double v0, double vf, double t){
        return v0+(vf-v0)/(2*Math.pow(T/2, n))*Math.pow(t, n);
    }

    private double quad(boolean fast, double input){
        if(input == 0){return 0;}

        double min = x_sqr_movement_params[0], max = x_sqr_movement_params[1];
        if(fast){
            min = x_sqr_movement_params[2];
            max = x_sqr_movement_params[3];
        }
        double b = min;
        double m = max - min;

        if(input < 0){
            return -m * input * input - b;
        }else{
            return m * input * input + b;
        }
    }

    private double lin(boolean fast, double input) {
        if (input > -ctrl_deadzone && input < ctrl_deadzone) {
            return 0;
        }
        double min = x_sqr_movement_params[0], max = x_sqr_movement_params[1];
        if (fast) {
            min = x_sqr_movement_params[2];
            max = x_sqr_movement_params[3];
        }

        double b = min;
        double m = (max - min) / (1 - ctrl_deadzone);
        if (input < 0) {
            return m * input - b;
        } else {
            return m * input + b;
        }
    }
}
