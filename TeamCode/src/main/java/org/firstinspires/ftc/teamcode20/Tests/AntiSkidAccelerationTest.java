package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp
public class AntiSkidAccelerationTest extends BaseAuto {
    private double[] params = {0.35, 1, 1000};
    private String[] paramNames =   {"startSpd", "goalSpd", "to1spd_MS"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb, y, APrimed = false;
    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
        initIMU();
        initLogger("antiSkid"+System.currentTimeMillis()+".csv");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2.setPhoneChargeEnabled(true);
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
            resetY1Odometry();
            resetXOdometry();
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            antiSkidAccelerationX(params[0], params[1], params[2]);
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

        telemetry.addData("parameters",params[0]+", "+params[1]+", "+params[2]);
        telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
        telemetry.addData("enc X", getXOdometry());
        telemetry.update();
    }

    private void antiSkidAccelerationX(double start, double goal, double accTime){
        writeLogHeader("start="+start+",goal="+goal+",acc time="+accTime+",batt "+hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2").read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)+"V");
        writeLogHeader("time,delta,current,odometry,wheel");
        ElapsedTime t = new ElapsedTime();
        double duration = accTime * Math.abs(goal - start), delta;
        setNewGyro0();
        double currentPower = start;
        setAllDrivePowerG(-currentPower,currentPower,-currentPower,currentPower);
        wait(100);
        t.reset();
        while(!this.gamepad1.b){
            delta = t.milliseconds()/duration;
            Log.i("AntiSlipX","delta = "+delta+", current = "+currentPower);
            currentPower  = start + delta;
            if(currentPower > goal){
                break;
            }
            setAllDrivePowerG(-currentPower,currentPower,-currentPower,currentPower);
            writeLog(t.milliseconds()+","+delta+","+currentPower+","+getXOdometry()+","+LF.getCurrentPosition());
        }
        wait(300);
        setAllDrivePower(0);
    }

    @Override
    public void stop() {
        stopLog();
        super.stop();
    }
}
