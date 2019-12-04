package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;
@Autonomous(name = "blue")

public class BlueAuto extends BaseAuto {
    private double speed;

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
        grabber_extender.setTargetPosition(0);
        grabber_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        speed=0.2;
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
        telemetry.addData("35",0);
        telemetry.update();
        while ( 4.3<left.getDistance(DistanceUnit.INCH) && 4.3<right.getDistance(DistanceUnit.INCH)){
            setAllDrivePowerG(-speed,-speed,speed,speed);
        }
        setAllDrivePower(0);
        grabber.setPosition(.2);
        grabber_extender.setTargetPosition(grabber_extender.getCurrentPosition()+30);
        moveInchesG(0,-2.5,speed);
        setAllDrivePower(0);
        turn(90,0.4,2);
        setNewGyro0();
        moveInchesG(0,60,0.3);
        requestOpModeStop();
    }
}