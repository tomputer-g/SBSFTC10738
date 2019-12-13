package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode20.TractionControl;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp
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
        initVuforiaWebcam();
        rangeSensorSide = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side");
        speed=0.3;
        y = 0;
        x = 0;

        // 三天之内刹了你();
    }

    @Override
    public void loop(){
        if(zheng(this.gamepad1.dpad_left,eee))x-=12;
        if(zheng(this.gamepad1.dpad_right,fff))x+=12;
        if(zheng(this.gamepad1.dpad_up,ee))y+=12;
        if(zheng(this.gamepad1.dpad_down,ff))y-=12;
        if(zheng(this.gamepad1.y,m))speed+=.1;
        if(zheng(this.gamepad1.a,mm))speed-=.1;

        if(zheng(this.gamepad1.b,f))setNewGyro0();
        if(zheng(this.gamepad1.right_bumper,bF)){
            /*
            moveInchesG(x,y,speed);
            moveInchesG(-x,-y,speed);
            setNewGyro(90);
            setAllDrivePowerG(0);
            ElapsedTime t=new ElapsedTime();
            while(t.milliseconds()<5000){
                setAllDrivePowerG(-speed,-speed,speed,speed);
                telemetry.addData("imuHeading: ",imuHeading);
                telemetry.addData("imuOffset: ",imuOffset);
                telemetry.update();
            }
             */
            ElapsedTime t=new ElapsedTime();
            targetsSkyStone.activate();
            VuforiaTrackable trackable = allTrackables.get(11);
            while(t.milliseconds()<5000) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    if (trackable.getName().equals("Rear Perimeter 1")) {
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addLine("Turn " + (int) Math.abs(rotation.thirdAngle - 90) + (rotation.thirdAngle - 90 > 0 ? "deg. CW" : "deg. CCW"));
                        VectorF translation = lastLocation.getTranslation();
                        double disty = translation.get(1)/mmPerInch;
                        double distx = translation.get(0)/mmPerInch;
                        double distz = translation.get(2)/mmPerInch;
                        telemetry.addData("x: ",distx);
                        telemetry.addData("y: ",disty);
                        telemetry.addData("z: ",distz);
                    }
                }
            }
            shutdownVuforia();
        }

        telemetry.addData("x: ",x);
        telemetry.addData("y: ",y);
        telemetry.addData("speed: ",speed);
        telemetry.update();
    }
}
