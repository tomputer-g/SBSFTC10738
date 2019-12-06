package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class TeleOp_ScaledDrive extends BaseOpMode {

    private final double ctrl_deadzone = 0.2;
    private double linear_proportion = 1;
    @Override
    public void init() {
        initDrivetrain();
    }

    @Override
    public void init_loop() {
        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            linear_proportion -= 0.05;
            if(linear_proportion < 0.2)
                linear_proportion = 0.2;
        }else if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            linear_proportion += 0.05;
            if(linear_proportion > 0.8)
                linear_proportion = 0.8;
        }

        telemetry.addLine("Linear: "+linear_proportion + ", Rotational: " + (1-linear_proportion));
        telemetry.update();
    }

    @Override
    public void loop() {
        scaledMove(linear(-this.gamepad1.left_stick_x, 0.2, linear_proportion), linear(-this.gamepad1.left_stick_y, 0.2, linear_proportion), linear(-this.gamepad1.right_stick_x, 0.2, (1-linear_proportion)));

        if(this.gamepad1.dpad_left){
            while(this.gamepad1.dpad_left);
            linear_proportion -= 0.05;
            if(linear_proportion < 0.2)
                linear_proportion = 0.2;
        }else if(this.gamepad1.dpad_right){
            while(this.gamepad1.dpad_right);
            linear_proportion += 0.05;
            if(linear_proportion > 0.8)
                linear_proportion = 0.8;
        }

        telemetry.addLine("Linear: "+linear_proportion + ", Rotational: " + (1-linear_proportion));
        telemetry.update();

    }

    private double linear(double input, double minLimit, double maxLimit){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}

        double m = (maxLimit - minLimit)/(1-ctrl_deadzone);
        if(input < 0)
            return m * input - minLimit;
        return m * input + minLimit;
    }

    private void scaledMove(double vx, double vy, double vr){
        telemetry.addLine("vX: "+to3d(vx)+", vY: "+to3d(vy)+", vR: "+to3d(vr));
        double[] speeds = {vx - vy + vr, -vy - vx + vr, vx + vy + vr, -vx + vy + vr};
        double absMax = 0;
        for(double d : speeds)
            absMax = Math.max(Math.abs(d),absMax);
        if(absMax <= 1){
            setAllDrivePower(speeds[0], speeds[1], speeds[2], speeds[3]);
            telemetry.addData("vLF",to3d(speeds[0]));
            telemetry.addData("vLB",to3d(speeds[1]));
            telemetry.addData("vRF",to3d(speeds[2]));
            telemetry.addData("vRB",to3d(speeds[3]));
        }else{
            telemetry.addLine("SCALED power: max was "+absMax);
            telemetry.addLine("vLF: "+to3d(speeds[0])+" -> "+to3d(speeds[0]/absMax));
            telemetry.addLine("vLB: "+to3d(speeds[1])+" -> "+to3d(speeds[1]/absMax));
            telemetry.addLine("vRF: "+to3d(speeds[2])+" -> "+to3d(speeds[2]/absMax));
            telemetry.addLine("vRB: "+to3d(speeds[3])+" -> "+to3d(speeds[3]/absMax));
            setAllDrivePower(speeds[0]/absMax, speeds[1]/absMax, speeds[2]/absMax,speeds[3]/absMax);
        }
    }

}
