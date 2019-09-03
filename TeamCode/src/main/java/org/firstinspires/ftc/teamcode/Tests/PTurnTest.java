package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BaseAuto;
@Disabled
@TeleOp
public class PTurnTest extends BaseAuto {
    private final double THRESHOLD_90 = 16.5, THRESHOLD_45 = 15; //turning tolerance
    private final double P_TURN = 0.25;
    private final double TURN_SPEED = 0.3;
    private double value = 180 + 52;
    @Override
    public void init() {
        super.init();
        telemetry.setAutoClear(true);
    }

    @Override public void loop() {
        getHeading();
        if(this.gamepad1.a){
            gyroTurn(imuHeading + 90, THRESHOLD_90);
        }else if(this.gamepad1.b){
            gyroTurn(imuHeading - 90, THRESHOLD_90);
        }else if(this.gamepad1.x){
            gyroTurn(imuHeading + 45, THRESHOLD_45);
        }else if(this.gamepad1.y){
            gyroTurn(imuHeading - 45, THRESHOLD_45);
        }
        else if(this.gamepad1.left_bumper){
            while (this.gamepad1.left_bumper);
            turn(value,0.15,3);
        }
        else if(this.gamepad1.dpad_up){
            while(this.gamepad1.dpad_up);
            value+=1;
        }
        else if(this.gamepad1.dpad_down){
            while(this.gamepad1.dpad_down);
            value-=1;
        }
        telemetry.addData("cur angle: ", value);
    }

    private void gyroTurn (double angle, double threshold) {
        while (!onHeading(TURN_SPEED, angle, P_TURN, threshold)) {
            telemetry.update();
        }

    }

    private boolean onHeading(double turnSpeed, double angle, double PCoeff, double threshold) {
        double   error;
        double   steer;
        boolean  onTarget = false;
        double speed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            speed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            speed  = turnSpeed * steer;
        }

        // Send desired speeds to motors.
        mecanumTurn(speed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Actual", imuHeading);
        telemetry.addData("Err", "%5.2f", error);
        telemetry.addData("Speed.", "%5.2f", speed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {
        double robotError;
        getHeading();
        robotError = targetAngle - imuHeading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private void mecanumTurn(double vr){
        LF.setPower(vr);
        LB.setPower(vr);
        RF.setPower(vr);
        RB.setPower(vr);
    }

}
