package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class BlueAuto extends TractionControl {
    private double speed = 0.4;
    @Override
    public void init() {
        showTelemetry = false;
        initDrivetrain();
        initIMU();
        initGrabber();
        initLinSlide();
        initPlatformGrabber();
        initVuforia();
        initSensors();
        setNewGyro0();
        speed=0.3;
    }
    @Override
    public void loop() {
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones

        //initialization
        //grabber_extender.setPower(1);
        wait(400);
        //grabber_extender.setPower(0.0);
        grabber.setPosition(grabber_open);
        platform_grabber.setPower(1);
        wait(300);
        platform_grabber.setPower(0.0);
        moveInchesG(0,15,0.3);
        if(showTelemetry)telemetry.clear();
        int pos = skystonePosition();
        shutdownVuforia();

        int shift;
        if(pos == 1){shift = 0;}
        else if (pos == 0){
            moveInchesG(-8,0,0.4);
            shift=10;
        }
        else {
            moveInchesG(8, 0, 0.4);
            shift=-8;
        }

        //move to the first blocc
        ElapsedTime p = new ElapsedTime();
        reset_ENCODER();
        setMode_RUN_WITHOUT_ENCODER();
        while ( (ymult*8>Math.abs(LB.getCurrentPosition())) && 1.3 < (left.getDistance(DistanceUnit.INCH)) && (1.3 < right.getDistance(DistanceUnit.INCH)) && p.milliseconds()<1500){
            setAllDrivePowerG(-0.25, -0.25, 0.25, 0.25);
        }
        wait(600);

        //grab 1st block
        grabber.setPosition(.55);
        setAllDrivePower(0.0);

        //move back
        moveInchesG(0,-8,0.3);

        //move forward & approach foundation
        turn(90, 0.4, 4);
        setNewGyro(90);
        p.reset();
        while(21.4<rangeSensorFront.getDistance(DistanceUnit.INCH)||p.milliseconds()<3000){
            setAllDrivePowerG(-speed,-speed,speed,speed);
        }
        setAllDrivePower(0.0);
        setAllDrivePowerG(-.35,.35,-.35,.35);
        wait(1500);


        //move foundation
        platform_grabber.setPower(-.8);
        wait(200);
        turn(90, 0.67, 5);
        /*
        //winston attempt
        while (!near(getHeading(),90,3)) setAllDrivePower(-0.6,0.2,0.8,-0.4);
        */

        //drag foundation
        setNewGyro(180);
        double koe=1;
        while(13<rangeSensorFront.getDistance(DistanceUnit.INCH)){
            setAllDrivePowerG(koe*(0.25-0.55+0.37),koe*(0.25-0.55-0.37),koe*(0.25+0.55+0.37),koe*(0.22+0.5-0.37)); //turn+f0rwrd+side
        }
        setAllDrivePower(0.0);

        //align to the right wall
        while(30>rangeSensorFront.getDistance(DistanceUnit.INCH)){
            //telemetry.addData("Side",rangeSensorSide.getDistance(DistanceUnit.INCH));
            //telemetry.update();
            setAllDrivePowerG(0.5,-0.5,0.5,-0.5);
        }
        setAllDrivePower(0.0);

        //turn and drop the block
        platform_grabber.setPower(1);
        wait(300);
        platform_grabber.setPower(0.0);
        moveInchesG(-2.3,0,0.4);
        turn(-90,0.5,8);
        L1.setPower(-0.35);
        L2.setPower(0.35);
        wait(500);
        //grabber_extender.setPower(-1);
        wait(800);
        setAllDrivePower(0.0);
        //grabber_extender.setPower(0.0);
        L1.setPower(0.0);
        L2.setPower(0.0);
        grabber.setPosition(grabber_open);
        L1.setPower(0.35);
        L2.setPower(-0.35);
        wait(1200);
        L1.setPower(0.0);
        L2.setPower(0.0);

        //park
        setNewGyro(90);
        moveInchesG(0, -19, 0.5);
        requestOpModeStop();
    }
}
