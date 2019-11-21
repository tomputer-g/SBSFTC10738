package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.teamcode20.TractionControl;

import static java.lang.Math.sqrt;

@TeleOp
public class MiscTest extends TractionControl {
    double speed,x,y,pc, side_distance, pc_side;
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

        speed=0.35;
        y = 0;
        x = 12;
        pc = .8;
        pc_side = 1;
        side_distance = 6;
       // 三天之内刹了你();
    }

    @Override
    public void loop(){
        //x+ left x- right y+ forward y- backward
        getHeading();
        if(整(this.gamepad1.y,e))speed-=0.05;
        if(整(this.gamepad1.a,f))speed+=0.05;
        if(整(this.gamepad1.dpad_up,ee))y++;
        if(整(this.gamepad1.dpad_down,ff))y--;
        if(整(this.gamepad1.dpad_left,eee))x++;
        if(整(this.gamepad1.dpad_right,fff))x--;
        if(整(this.gamepad1.x,mm))pc+=0.2;
        if(整(this.gamepad1.b,mmm))pc-=0.2;
        telemetry.addData("speed: ","%.2f",speed);
        telemetry.addData("x:", x);
        telemetry.addData("y: ", y);
        telemetry.addData("LF",LF.getCurrentPosition());
        telemetry.addData("LB",LB.getCurrentPosition());
        telemetry.addData("RF",RF.getCurrentPosition());
        telemetry.addData("RB",RB.getCurrentPosition());
        telemetry.addData("Heading",imuHeading);
        telemetry.addData("pc",pc);
        if(整(this.gamepad1.back,jk)){
            setNewGyro0();
        }
        if(整(this.gamepad1.right_bumper,bF)) {
            //moveInches(0,y,speed);
            t.reset();
            double a,b,c,d,adjustSide;
            while (t.milliseconds() < 2500) {
                setAllDrivePowerG(-speed, -speed, speed, speed, pc);
                a = LF.getPower(); b = LB.getPower(); c = RF.getPower(); d = RB.getPower();
                adjustSide = pc_side*(rangeSensorSide.getDistance(DistanceUnit.INCH) - side_distance);
                adjustSide = Math.min(0.15,Math.max(-0.15,adjustSide));
                setAllDrivePower(a-adjustSide,b+adjustSide,c-adjustSide,d+adjustSide);
                //sideway:
                //setAllDrivePowerG(-speed, speed, -speed, speed, pc);

                telemetry.addData("Heading", imuHeading);
                telemetry.addData("t",t.milliseconds());
                telemetry.update();
            }

            setAllDrivePower(-LF.getPower()/Math.abs(LF.getPower()),-LB.getPower()/Math.abs(LB.getPower()),-RF.getPower()/Math.abs(RF.getPower()),-RB.getPower()/Math.abs(RB.getPower()));
            wait(70);
            setAllDrivePower(0);
        }
        if(整(this.gamepad1.left_bumper,lF))
            speed*=-1;
            //turn(90,speed,0.5);
        if (整(this.gamepad1.start, m))
            moveInches(x,y,speed);
        telemetry.update();
    }
}
