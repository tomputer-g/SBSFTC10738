package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.TractionControl;

@TeleOp(group = "Test", name = "druagtest")
public class MiscTest extends TractionControl {
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
        //rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        initPlatformGrabber();
        initSensors();
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        speed=0.5;
        y = 15;
        x = 0;
        GYRO_kp = .8;
        kd = 0;
        kp = .6;
        side_distance = 6;
       // 三天之内刹了你();
    }

    @Override
    public void loop(){
        //x+ left x- right y+ forward y- backward
        getHeading();
        if(zheng(this.gamepad1.y,e))speed-=0.05;
        if(zheng(this.gamepad1.a,f))speed+=0.05;
        if(zheng(this.gamepad1.dpad_up,ee))y++;
        if(zheng(this.gamepad1.dpad_down,ff))y--;
        if(zheng(this.gamepad1.dpad_left,eee))kd+=0.1;
        if(zheng(this.gamepad1.dpad_right,fff))kd-=0.1;
        if(zheng(this.gamepad1.x,mm))side_distance++;
        if(zheng(this.gamepad1.b,mmm))side_distance--;
        telemetry.addData("speed: ","%.2f",speed);
        //telemetry.addData("x:", x);
        //telemetry.addData("y: ", y);
        //telemetry.addData("LF",LF.getCurrentPosition());
        //telemetry.addData("LB",LB.getCurrentPosition());
        //telemetry.addData("RF",RF.getCurrentPosition());
        //telemetry.addData("RB",RB.getCurrentPosition());
        //telemetry.addData("side_sensor val ", "%.2f",rangeSensorSide.getDistance(DistanceUnit.INCH));
        //telemetry.addData("side_dis", side_distance);
        //telemetry.addData("Heading","%.2f",imuHeading);
        //telemetry.addData("GYRO_kp", GYRO_kp);
        //telemetry.addData("KP", "%.1f",kp);
        telemetry.addData("y", "%.1f",y);

        if(zheng(this.gamepad1.back,jk)) setNewGyro0();
        if(zheng(this.gamepad1.start,bF)) {
            //moveInches(0,y,speed);
            t.reset();
            double p, dd,cur=0,pre=0, error;
            double a=-speed,b=-speed,c=speed,d=speed;
            while (t.milliseconds() < 2500) {
                cur= rangeSensorSide.getDistance(DistanceUnit.INCH) - side_distance;
                p = Math.min(0.23,Math.max(-0.23, kp *cur));
                dd = kd*(cur-pre);
                error = p+dd;
                pre = cur;
                setAllDrivePowerG(a-error,b+error,c-error,d+error);
                //sideway:
                //setAllDrivePowerG(-speed, speed, -speed, speed, GYRO_kp);
                telemetry.addData("side_sensor val ", "%.2f",rangeSensorSide.getDistance(DistanceUnit.INCH));
                telemetry.addData("Heading", imuHeading);
                telemetry.addData("t",t.milliseconds());
                telemetry.update();
            }
            //brakeTD(1);
            //setAllDrivePower(-LF.getPower()/Math.abs(LF.getPower()),-LB.getPower()/Math.abs(LB.getPower()),-RF.getPower()/Math.abs(RF.getPower()),-RB.getPower()/Math.abs(RB.getPower()));
            //wait(70);
            setAllDrivePower(0);
        }
        if(zheng(this.gamepad1.left_bumper,lF))
            speed*=-1;
            //
        if (zheng(this.gamepad1.right_bumper, m)){
            platform_grabber.setPower(-.8);
            turn(90, 0.67, 5);
            setNewGyro(90);
            while(9<rangeSensorFront.getDistance(DistanceUnit.INCH)){
                setAllDrivePowerG(0.35-speed,0.35-speed,0.35+speed,0.35+speed);
                telemetry.addData("Front",rangeSensorFront.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
            //setAllDrivePowerG(0);
            //wait(1000);
            setAllDrivePower(0);
            moveInchesG(-15,0,0.72);
            platform_grabber.setPower(0);
        }
        telemetry.update();
    }
}
