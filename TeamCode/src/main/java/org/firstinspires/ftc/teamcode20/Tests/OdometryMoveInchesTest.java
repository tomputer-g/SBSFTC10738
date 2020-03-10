package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.openftc.revextensions2.ExpansionHubEx;
@TeleOp
public class OdometryMoveInchesTest extends BaseAuto {
    /*
    target is 100 inches

    0.3 speed: P = 1,       D = 0.12,   result: +1/32 in
    0.6 speed: P = 0.075,   D = 1.4E-2, result: 0 in.
    0.9 speed: P = 0.0325,  D = 7.3E-3, result: spin +- 1/4 in
     */
    private double[] params =       {0,  0,     0,       0.9,        90,           0.915};
    private String[] paramNames =   {"P",   "I",    "D",    "speed",    "targetInches","k"};
    private int currentSelectParamIndex = 3;
    private boolean l, r, u, d, lb, rb, y, APrimed = false, x = false, platformGrabbed = false;

    protected final double odometryEncYPerInch = 1324.28, odometryEncXPerInch = 1316.38;

    @Override //chiggas
    public void runOpMode() throws InterruptedException {
        initHubs();
        initDrivetrain();
        initPlatformGrabber();
        initOdometry();
        initIMU();
        initLogger("OdoMoveInchesX"+System.currentTimeMillis()+".csv");
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
                resetY1Odometry();
                resetXOdometry();
                setNewGyro0();
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                moveInchesGOY_XF(params[4], params[3],params[5]);
            }

            if(this.gamepad1.left_bumper){lb = true;}if(!this.gamepad1.left_bumper && lb){
                lb = false;
                currentSelectParamIndex--;
                if(currentSelectParamIndex < 0){
                    currentSelectParamIndex = params.length - 1;
                }
            }
            if(this.gamepad1.right_bumper){rb = true;}if(!this.gamepad1.right_bumper && rb){
                rb = false;
                currentSelectParamIndex++;
                if(currentSelectParamIndex >= params.length){
                    currentSelectParamIndex = 0;
                }
            }
            if(this.gamepad1.dpad_left){l = true;}if(!this.gamepad1.dpad_left && l){
                l = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] - 1) * 1E9) / 1E9;

            }
            if(this.gamepad1.dpad_right){r = true;}if(!this.gamepad1.dpad_right && r){
                r = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] + 1) * 1E9) / 1E9;

            }
            if(this.gamepad1.dpad_up){u = true;}if(!this.gamepad1.dpad_up && u){
                u = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] * 10.0) * 1E9) / 1E9;

            }
            if(this.gamepad1.dpad_down){d = true;}if(!this.gamepad1.dpad_down && d){
                d = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] / 10.0) * 1E9) / 1E9;

            }
            if(this.gamepad1.y){y = true;}if(!this.gamepad1.y && y){
                y=false;
                params[currentSelectParamIndex] = -params[currentSelectParamIndex];
            }

            if(this.gamepad1.x) {
                x = true;
            }if(!this.gamepad1.x && x){
                x = false;
                if(platformGrabbed){//already held
                    platformGrabbed = false;
                    platform_grabber.setPower(1);
                    Thread.sleep(100);
                    platform_grabber.setPower(0);
                }else{
                    platformGrabbed = true;
                    platform_grabber.setPower(-0.4);
                }
            }
            telemetry.addData("parameters",params[3]+", "+params[4]+","+params[5]);
            telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            telemetry.addData("enc X", getXOdometry());
            telemetry.addData("enc Y1",getY1Odometry());
            telemetry.update();
        }
        stopLog();
    }

}
