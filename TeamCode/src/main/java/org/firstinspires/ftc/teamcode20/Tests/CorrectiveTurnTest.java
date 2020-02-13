package org.firstinspires.ftc.teamcode20.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class CorrectiveTurnTest extends BaseAuto {
    private double[] params = {0.3,90,1E-4,1E-5,0.5};//,90.0,0};
    private String[] paramNames = {"speed","angle","P","D","rampUpS"};//, "angle", "accTime"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb, a, y;


    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initLogger("CorrectiveTurnTest"+System.currentTimeMillis()+".csv");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        waitForStart();
        while(opModeIsActive()){
            if(this.gamepad1.a){a = true;}if(!this.gamepad1.a && a){
                a = false;
                correctiveTurn(params[1], params[0]);
                //correctiveTurn(params[1],params[0],params[2]);
            }
        /*if(this.gamepad1.y){y = true;}if(!this.gamepad1.y && y){
            y = false;
            runToPositionSpeed(params[1]);
        }

         */
            if(this.gamepad1.b){
                LF.setPower(0);
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LB.setPower(0);
                LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RF.setPower(0);
                RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RB.setPower(0);
                RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] - 1) * 1E6) / 1E6;

            }
            if(this.gamepad1.dpad_right){r = true;}if(!this.gamepad1.dpad_right && r){
                r = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] + 1) * 1E6) / 1E6;

            }
            if(this.gamepad1.dpad_up){u = true;}if(!this.gamepad1.dpad_up && u){
                u = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] * 10.0) * 1E6) / 1E6;

            }
            if(this.gamepad1.dpad_down){d = true;}if(!this.gamepad1.dpad_down && d){
                d = false;
                params[currentSelectParamIndex] = Math.round((params[currentSelectParamIndex] / 10.0) * 1E6) / 1E6;

            }

            telemetry.addData("parameters",params[0]+", "+params[1]+", "+params[2]+","+params[3]+","+params[4]);
            telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            telemetry.update();
        }
        stopLog();
    }


    private void correctiveTurn(double angle, double OCspeed){
        writeLogHeader("P="+params[2]+",D="+params[3]+",target="+OCspeed+",angle"+angle+",batt "+hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)+"V");
        setNewGyro0();
        double multiplier = 0;
        double currentError = 0, lastError = 0;
        double lastSpeed = 0;
        ElapsedTime t = new ElapsedTime();
        while(!this.gamepad1.b){
            double currentHeading = getHeading();
            currentError = currentHeading - angle;
            multiplier = -Math.max(-1, Math.min(1, params[2] * currentError + params[3] * (lastError - currentError)*1000/t.milliseconds()));
            //lastSpeed = lastSpeed + multiplier * OCspeed;
            setAllDrivePower(multiplier * OCspeed);
            t.reset();
            lastError = currentError;//CCW=+
        }
        setAllDrivePower(0);
    }


    private void correctiveTurnSpeed(double targetSpeed){
        writeLogHeader("P="+params[2]+",D="+params[3]+",target="+targetSpeed+",batt "+hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)+"V");
        writeLogHeader("LF pos,LB pos,RF pos,RB pos,LF spd,LB spd,RF spd,RB spd,LF pwr,LB pwr,RF pwr,RB pwr");
        LF.setPower(0);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setPower(0);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setPower(0);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setPower(0);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double[] previousPos = {0,0,0,0};
        double[] currentSpeed = {0,0,0,0};
        double[] setPower = {0,0,0,0};
        double[] lastSpeed = {0,0,0,0};
        double rampupSpeed = 0;
        ElapsedTime t = new ElapsedTime();
        /*
        double rampUpSpeed = 0;
        while(!this.gamepad1.b){
            wait(50);
            double dT = t.milliseconds() / 1000;
            rampUpSpeed += 2 * dT * targetSpeed;
            if((targetSpeed < 0? rampUpSpeed<targetSpeed : rampUpSpeed>targetSpeed)){
                rampUpSpeed = targetSpeed;
                break;
            }
            RevBulkData bulk = hub2.getBulkInputData();
            currentSpeed[0] = ( bulk.getMotorCurrentPosition(LF) - previousPos[0] )/dT;
            currentSpeed[1] = ( bulk.getMotorCurrentPosition(LB) - previousPos[1] )/dT;
            currentSpeed[2] = ( bulk.getMotorCurrentPosition(RF) - previousPos[2] )/dT;
            currentSpeed[3] = ( bulk.getMotorCurrentPosition(RB) - previousPos[3] )/dT;
            previousPos[0] = bulk.getMotorCurrentPosition(LF);
            previousPos[1] = bulk.getMotorCurrentPosition(LB);
            previousPos[2] = bulk.getMotorCurrentPosition(RF);
            previousPos[3] = bulk.getMotorCurrentPosition(RB);
            for(int i = 0;i < 4; i++){
                double dPower = 1E-4 * (rampUpSpeed - currentSpeed[i]) + 1E-5 * (lastSpeed[i] - currentSpeed[i]);
                lastSpeed[i] = setPower[i];
                setPower[i] += dPower;
                setPower[i] = Math.min(1,Math.max(-1, setPower[i]));
            }

            LF.setPower(setPower[0]);
            LB.setPower(setPower[1]);
            RF.setPower(setPower[2]);
            RB.setPower(setPower[3]);
            writeLog(previousPos[0]+","+previousPos[1]+","+previousPos[2]+","+previousPos[3]+","+currentSpeed[0]+","+currentSpeed[1]+","+currentSpeed[2]+","+currentSpeed[3]+","+setPower[0]+","+setPower[1]+","+setPower[2]+","+setPower[3]);
            Log.i("CorrectiveTurn-rampup", "currentSpeed="+currentSpeed[0]+","+currentSpeed[1]+","+currentSpeed[2]+","+currentSpeed[3]
                    +", setPower="+setPower[0]+","+setPower[1]+","+setPower[2]+","+setPower[3]+",rampup="+rampUpSpeed);
            t.reset();
        }

         */
        /*while(!this.gamepad1.b){
            wait(50);
            double dT = t.milliseconds() / 1000;
            RevBulkData bulk = hub2.getBulkInputData();
            currentSpeed[0] = ( bulk.getMotorCurrentPosition(LF) - previousPos[0] )/dT;
            currentSpeed[1] = ( bulk.getMotorCurrentPosition(LB) - previousPos[1] )/dT;
            currentSpeed[2] = ( bulk.getMotorCurrentPosition(RF) - previousPos[2] )/dT;
            currentSpeed[3] = ( bulk.getMotorCurrentPosition(RB) - previousPos[3] )/dT;
            previousPos[0] = bulk.getMotorCurrentPosition(LF);
            previousPos[1] = bulk.getMotorCurrentPosition(LB);
            previousPos[2] = bulk.getMotorCurrentPosition(RF);
            previousPos[3] = bulk.getMotorCurrentPosition(RB);
            rampupSpeed += dT * targetSpeed/params[4];//speed up
            if(Math.abs(rampupSpeed) > Math.abs(targetSpeed)){
                break;
            }
            for(int i = 0;i < 4; i++){
                double dPower = 1E-4 * (rampupSpeed - currentSpeed[i]) + 1E-5 * (lastSpeed[i] - currentSpeed[i]);
                lastSpeed[i] = setPower[i];
                setPower[i] += dPower;
                setPower[i] = Math.min(1,Math.max(-1, setPower[i]));
            }
            setAllDrivePower(setPower[0], setPower[1], setPower[2], setPower[3]);
            writeLog(previousPos[0]+","+previousPos[1]+","+previousPos[2]+","+previousPos[3]+","+currentSpeed[0]+","+currentSpeed[1]+","+currentSpeed[2]+","+currentSpeed[3]+","+setPower[0]+","+setPower[1]+","+setPower[2]+","+setPower[3]);
            Log.i("CorrectiveTurn", "currentSpeed="+currentSpeed[0]+","+currentSpeed[1]+","+currentSpeed[2]+","+currentSpeed[3]
                    +", setPower="+setPower[0]+","+setPower[1]+","+setPower[2]+","+setPower[3]);
            t.reset();
        }

         */

        while(!this.gamepad1.b){
            wait(50);
            double dT = t.milliseconds() / 1000;
            RevBulkData bulk = hub2.getBulkInputData();
            currentSpeed[0] = ( bulk.getMotorCurrentPosition(LF) - previousPos[0] )/dT;
            currentSpeed[1] = ( bulk.getMotorCurrentPosition(LB) - previousPos[1] )/dT;
            currentSpeed[2] = ( bulk.getMotorCurrentPosition(RF) - previousPos[2] )/dT;
            currentSpeed[3] = ( bulk.getMotorCurrentPosition(RB) - previousPos[3] )/dT;
            previousPos[0] = bulk.getMotorCurrentPosition(LF);
            previousPos[1] = bulk.getMotorCurrentPosition(LB);
            previousPos[2] = bulk.getMotorCurrentPosition(RF);
            previousPos[3] = bulk.getMotorCurrentPosition(RB);
            for(int i = 0;i < 4; i++){
                double dPower = 1E-4 * (targetSpeed - currentSpeed[i]) + 1E-5 * (lastSpeed[i] - currentSpeed[i]);
                lastSpeed[i] = setPower[i];
                setPower[i] += dPower;
                setPower[i] = Math.min(1,Math.max(-1, setPower[i]));
            }
            setAllDrivePower(setPower[0], setPower[1], setPower[2], setPower[3]);
            writeLog(previousPos[0]+","+previousPos[1]+","+previousPos[2]+","+previousPos[3]+","+currentSpeed[0]+","+currentSpeed[1]+","+currentSpeed[2]+","+currentSpeed[3]+","+setPower[0]+","+setPower[1]+","+setPower[2]+","+setPower[3]);
            Log.i("CorrectiveTurn", "currentSpeed="+currentSpeed[0]+","+currentSpeed[1]+","+currentSpeed[2]+","+currentSpeed[3]
                                            +", setPower="+setPower[0]+","+setPower[1]+","+setPower[2]+","+setPower[3]);
            t.reset();
        }

    }

    /*private void runToPositionSpeed(double speed) {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setTargetPosition(2000+LF.getCurrentPosition());
        LB.setTargetPosition(2000+LF.getCurrentPosition());
        RF.setTargetPosition(2000+LF.getCurrentPosition());
        RB.setTargetPosition(2000+LF.getCurrentPosition());
        LF.setPower(speed);
        LB.setPower(speed);
        RF.setPower(speed);
        RB.setPower(speed);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void correctiveTurn(double angle, double speed, double accTime){
        setNewGyro0();

    }


     */
}
