package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;
@Autonomous(name = "сделай это синим цветом")

public class BlueAuto extends BaseAuto {
    private double speed;
    private boolean[] ca={true},cb={true};
    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initGrabber();
        initLinSlide();
        initVuforiaWebcam();
        initSensors();
        setNewGyro0();
        grabber.setPosition(1);
        grabber_extender.setPower(1);
        wait(100);
        grabber_extender.setPower(0);
        speed=0.25;
        telemetry.addLine("6/6 INIT FINISHED");
    }
    @Override
    public void init_loop(){}
    @Override
    public void internalPostInitLoop(){}
    @Override
    public void loop() {
        //go forward 1 floor mat (24")
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones
            while (5 < left.getDistance(DistanceUnit.INCH) && 5 < right.getDistance(DistanceUnit.INCH)) {
                setAllDrivePowerG(-speed, -speed, speed, speed);
            }
            setAllDrivePower(0);
            grabber.setPosition(.2);
            grabber_extender.setPower(-1);
            wait(60);
            grabber_extender.setPower(0);
            moveInchesG(0, -2.5, speed);
            setAllDrivePower(0);
            turn(90, 0.4, 1);
            setNewGyro0();
            moveInchesG(0, 60, 0.3);
            grabber.setPosition(1);
            wait(200);
            moveInchesG(0, -60, 0.3);
            turn(-90,0.4,1);
            grabber_extender.setPower(1);
            wait(60);
            grabber_extender.setPower(0);
            setNewGyro0();
            while (5 < left.getDistance(DistanceUnit.INCH) && 5 < right.getDistance(DistanceUnit.INCH))setAllDrivePowerG(-speed, -speed, speed, speed);
            setAllDrivePower(0);
            grabber.setPosition(.2);
            grabber_extender.setPower(-1);
            wait(60);
            grabber_extender.setPower(0);
            moveInchesG(0, -5, speed);
            setAllDrivePower(0);
            turn(90, 0.4, 1);
            setNewGyro0();
            moveInchesG(0, 70, 0.3);
            moveInchesG(0, -14, 0.3);
            requestOpModeStop();
    }
}