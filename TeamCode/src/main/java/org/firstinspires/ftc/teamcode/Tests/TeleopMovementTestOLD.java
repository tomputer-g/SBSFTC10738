package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseOpMode;

@TeleOp(name = "MovementTestOLD", group = "Tests")
@Disabled
public class TeleopMovementTestOLD extends BaseOpMode {
    private static int stateNum = -1, pastStateNum = -1;
    private static boolean new_ez_pz_operation_completed = false;
    private static final double[] movement_power_params = {.1, .3, 0.8};
    private static final double ctrl_deadzone = 0.2;
    private static ElapsedTime t_ez_pz;
    int movement ; // in Inches
    double speed;

    @Override
    public void init() {
        super.init();
        t_ez_pz = new ElapsedTime();
        movement = 72;
        speed = 0.5;
    }

    @Override
    public void loop() {
        super.loop();
        if (this.gamepad1.dpad_up)
        {
            while (this.gamepad1.dpad_up);
            movement += 2;
        }
        else if (this.gamepad1.dpad_down)
        {
            while(this.gamepad1.dpad_down);
            movement -= 2;
        }
        else if (this.gamepad1.y)
        {
            while(this.gamepad1.y);
            moveInches(0, movement,speed);
        }
        else if (this.gamepad1.a)
        {
            while(this.gamepad1.a);
            moveInches(0, -movement,speed);
        }
        else if (this.gamepad1.x)
        {
            while(this.gamepad1.x);
            moveInches(-movement, 0,speed);
        }
        else if (this.gamepad1.b)
        {
            while(this.gamepad1.b);
            moveInches(movement, 0,speed);
        }
        
        else if (this.gamepad1.left_bumper)
        {
            while(this.gamepad1.left_bumper);
            speed-=0.05;
        }
        else if (this.gamepad1.right_bumper)
        {
            while(this.gamepad1.right_bumper);
            speed+=0.05;
        }
        telemetry.addData("current speed: ","%.2f", speed);
        telemetry.addData("current movement distance: ", movement);
        telemetry.update();
    }
}
