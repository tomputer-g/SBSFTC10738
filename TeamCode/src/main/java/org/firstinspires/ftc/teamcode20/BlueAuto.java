package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
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
        grabber.setPosition(0);
        grabber_extender.setPower(1);
        wait(100);
        grabber_extender.setPower(0);
        speed=0.4;
        telemetry.setAutoClear(false);
    }
    @Override
    public void loop() {
        //go forward 1 floor mat (24")w
        //vuforia - recognize block & move to pick up
        //after pickup: turn 90 deg. move to platform, drop off
        //move to platform, drag into position, release
        //repeat until run out of time; first on other skystones
        //moveInches(-8,0,speed);
        moveInchesG(0,12,speed);
        telemetry.clear();
        int pos = skystonePosition();
        telemetry.addData("pos: ",pos);
        telemetry.update();
        if(pos == 1);
        else if (pos == 0) moveInchesG(-8,0,speed);
        else{}
        //vuforia

        //move to blocc
        while ((4 < left.getDistance(DistanceUnit.INCH)) && (4 < right.getDistance(DistanceUnit.INCH))) {
            setAllDrivePowerG(-speed, -speed, speed, speed);
            telemetry.addData("L: ", left.getDistance(DistanceUnit.INCH));
            telemetry.addData("R:", right.getDistance(DistanceUnit.INCH));
            telemetry.addData("power", LF.getPower());
            telemetry.addData("power", LB.getPower());

            telemetry.addData("power", RF.getPower());

            telemetry.addData("power", RB.getPower());

            telemetry.update();
        }
        setAllDrivePower(0);
        grabber.setPosition(.25);
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
        grabber.setPosition(0);
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
        grabber.setPosition(.25);
        grabber_extender.setPower(-1);
        wait(60);
        grabber_extender.setPower(0);
        moveInchesG(0, -5.5, speed);
        setAllDrivePower(0);
        turn(90, 0.4, 2);
        setNewGyro(90);
        moveInchesG(0, 84, 0.3);

        //drop
        grabber.setPosition(0);
        wait(200);

        //park
        moveInchesG(0, -23, 0.3);
        requestOpModeStop();
    }
}