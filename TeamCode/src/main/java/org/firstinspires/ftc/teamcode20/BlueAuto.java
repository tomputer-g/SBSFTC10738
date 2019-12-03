package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseAuto;
@Autonomous(group = "синий")

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
        speed=0.2;
    }

    @Override
    public void loop() {
        //go forward 1 floor mat (24")
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones
        while (4<left.getDistance(DistanceUnit.INCH)&&4<right.getDistance(DistanceUnit.INCH)){
            setAllDrivePowerG(-speed,-speed,speed,speed);
        }
        setAllDrivePower(0);
        moveInchesG(0,-2.5,speed);
        setAllDrivePower(0);
        turn(90,0.25,3);
        imuOffset=-90;
        moveInchesG(0,50,0.3);
        requestOpModeStop();
    }
}
