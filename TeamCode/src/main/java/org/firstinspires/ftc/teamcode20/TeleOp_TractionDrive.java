package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class TeleOp_TractionDrive extends TractionControl {

    private final double ctrl_deadzone = 0.2;
    private double linear_proportion = 1;
    private boolean[] primed = {true};
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
        if(zheng(this.gamepad1.left_bumper,primed)) brakeTD(1.0,10);

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


}

