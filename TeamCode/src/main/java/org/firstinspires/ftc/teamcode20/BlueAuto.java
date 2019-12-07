package org.firstinspires.ftc.teamcode20;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode20.BaseAuto;
@Autonomous
public class BlueAuto extends BaseAuto {
    private double speed = 0.4;
    private boolean[] ca={true},cb={true};
    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initGrabber();
        initLinSlide();
        initVuforiaWebcam();
        left = hardwareMap.get(Rev2mDistanceSensor.class,"left");
        right = hardwareMap.get(Rev2mDistanceSensor.class,"right");
        setNewGyro0();
        grabber.setPosition(grabber_open);
        grabber_extender.setPower(1);
        wait(100);
        grabber_extender.setPower(0);
    }
    @Override
    public void loop() {
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones
        moveInchesG(0,12,speed);
        telemetry.clear();
        int pos = skystonePosition();
        telemetry.addData("pos: ",pos);
        telemetry.update();
        if(pos == 0){ moveInchesG(-8,0,speed);}
        if(pos == 2){ moveInchesG(8,0, speed);}
        //vuforia
        // move to blocc
        setAllDrivePower(-speed, -speed, speed, speed);
        telemetry.addLine("set speed");
        telemetry.update();
        while ((4 < left.getDistance(DistanceUnit.INCH)) && (4 < right.getDistance(DistanceUnit.INCH)));
        setAllDrivePower(0);
        grabber.setPosition(grabber_closed);
        wait(250);
        grabber_extender.setPower(-1);
        wait(60);
        grabber_extender.setPower(0);

        moveInchesG(0, -3.5, speed);
        setAllDrivePower(0);
        turn(90, 0.37, 3);
        setNewGyro(90);
        moveInchesG(0, 60, 0.3);

        //drop
        grabber.setPosition(grabber_open);
        wait(200);

        //going bacc for the second blocc
        moveInchesG(0, -84, 0.3);
        turn(-90,0.37,3);
        grabber_extender.setPower(1);
        wait(60);
        grabber_extender.setPower(0);
        setNewGyro(0);

        //grab the second blocc
        while (4 < left.getDistance(DistanceUnit.INCH) && 4 < right.getDistance(DistanceUnit.INCH))setAllDrivePowerG(-speed, -speed, speed, speed);
        setAllDrivePower(0);
        grabber.setPosition(grabber_closed);
        grabber_extender.setPower(-1);
        wait(60);
        grabber_extender.setPower(0);
        moveInchesG(0, -5.5, speed);
        setAllDrivePower(0);
        turn(90, 0.4, 2);
        setNewGyro(90);
        moveInchesG(0, 84, 0.3);

        //drop
        grabber.setPosition(grabber_open);
        wait(200);

        //park
        moveInchesG(0, -23, 0.3);
        requestOpModeStop();
    }
}