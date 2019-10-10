package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


public class TeleOp_NewDrive extends BaseOpMode {

    protected static final double ctrl_deadzone = 0.2;
    protected static final double[] movement_power_params = {.25, 8/14, .25, 1};//TODO: scaling power makes spin bad


    @Override
    public void init() {
        initDrivetrain();
    }

    @Override
    public void loop() {
        move(custom_quad(this.gamepad1.left_trigger > 0.5,-this.gamepad1.left_stick_x), custom_quad(this.gamepad1.left_trigger > 0.5,-this.gamepad1.left_stick_y), custom_linear(this.gamepad1.left_trigger > 0.5,-this.gamepad1.right_stick_x)*0.6);
        telemetry.update();
    }

    private double custom_linear(boolean fast, double input){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}
        double min = movement_power_params[0], max = movement_power_params[1];
        if(fast){
            min = movement_power_params[2];
            max = movement_power_params[3];
        }
        double b = min, m = (max - min)/(1-ctrl_deadzone);
        if(input < 0)
            return m * input - b;
        return m * input + b;
    }

    private double custom_quad(boolean fast, double input){
        if(input > -ctrl_deadzone && input < ctrl_deadzone){return 0;}
        double min = movement_power_params[0], max = movement_power_params[1];
        if(fast){
            min = movement_power_params[2];
            max = movement_power_params[3];
        }
        double b = min, m = max - min;
        if(input < 0)
            return -m * input * input - b;
        return m * input * input + b;
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
