package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.TractionControl;

public class MoveTest extends TractionControl {
    double speed,x,y, GYRO_kp, side_distance, kp,kd;
    boolean[] bF={true}, lF = {true}, e = {true}, f = {true}, ee = {true}, ff = {true}, eee = {true}, fff = {true}, m = {true},mm={true},mmm={true},jk={true};
    ElapsedTime t=new ElapsedTime();
    ModernRoboticsI2cRangeSensor rangeSensorSide;
    private void 三天之内刹了你(){
        setAllDrivePower(1,1,-1,-1);
        wait(200);
        setAllDrivePower(0);
    }

    @Override
    public void init(){
        initIMU();
        initDrivetrain();
        rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");

        speed=0.5;
        y = 0;
        x = -72;

        // 三天之内刹了你();
    }

    @Override
    public void loop(){
        if(整(this.gamepad1.dpad_left,eee))x-=12;
        if(整(this.gamepad1.dpad_right,fff))x+=12;
        if(整(this.gamepad1.dpad_up,ee))y+=0.1;
        if(整(this.gamepad1.dpad_down,ff))y-=0.1;
        if(整(this.gamepad1.right_bumper,bF))moveInchesG(x,y,speed);
        telemetry.addData("x: ",x);
        telemetry.addData("y: ",y);
        telemetry.update();
    }
}
