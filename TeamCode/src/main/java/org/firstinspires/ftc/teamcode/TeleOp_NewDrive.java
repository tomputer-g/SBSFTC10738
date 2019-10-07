package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOp_NewDrive extends BaseOpMode {

    protected static final double ctrl_deadzone = 0.2;
    protected static final double[] movement_power_params = {.25, .6, .25, 1};

    @Override
    public void init() {
        initDrivetrain();
    }

    @Override
    public void loop() {
        move(custom_quad(this.gamepad1.left_trigger > 0.5,-this.gamepad1.left_stick_x), custom_quad(this.gamepad1.left_trigger > 0.5,-this.gamepad1.left_stick_y), custom_linear(this.gamepad1.left_trigger > 0.5,-this.gamepad1.right_stick_x)*0.6);
    }

    protected double custom_linear(boolean fast, double input){
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

    protected double custom_quad(boolean fast, double input){
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
}
