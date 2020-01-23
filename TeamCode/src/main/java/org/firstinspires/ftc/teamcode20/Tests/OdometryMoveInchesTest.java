package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    private double[] params =       {1E-4,  0,     0,       0.9,        -100,           0.8};
    private String[] paramNames =   {"P",   "I",    "D",    "speed",    "targetInches", "turnkP"};
    private int currentSelectParamIndex = 0;
    private boolean l, r, u, d, lb, rb, APrimed = false;

    protected final double odometryEncPerInch = 1324.28;

    @Override
    public void init() {
        initDrivetrain();
        initOdometry();
        initIMU();
        initLogger("PIDtest"+System.currentTimeMillis()+".csv");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
    }
    @Override
    public void loop() {
        if(this.gamepad1.a){APrimed = true;}if(APrimed && !this.gamepad1.a){ APrimed = false;
            resetYOdometry();
            resetXOdometry();
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            moveInchesGO(params[4],params[3]);
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

        telemetry.addData("parameters",params[0]+", "+params[1]+", "+params[2]+", "+params[3]+", "+params[4]);
        telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
        telemetry.addData("enc Y", getYOdometry());
        telemetry.addData("target", params[4] * odometryEncPerInch);
        telemetry.addLine("0.3 speed: P = 1,       D = 0.12\n" +
                                    "0.6 speed: P = 0.075,   D = 1.4E-2\n" +
                                    "0.9 speed: P = 0.0325,  D = 7.3E-3");
        telemetry.update();
    }


    @Override
    public void stop() {
        stopLog();
        super.stop();
    }

    protected void moveInchesGO(double yInch, double speed){
        if(yInch == 0)return;
        writeLogHeader("P="+params[0]+", I="+params[1]+", D="+params[2]+",speed="+speed+",target="+yInch * odometryEncPerInch+", batt"+hub2.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
        writeLogHeader("millis time,position,P,eff. P Power,I,D,eff. D Power,LF speed,LF counts,GyroDrift");
        ElapsedTime t = new ElapsedTime();
        int offsetY = getYOdometry();
        speed=Math.abs(speed);
        double multiply_factor;
        int odometryYGoal = offsetY + (int)(yInch * odometryEncPerInch);
        double vy = (yInch/Math.abs(yInch)*speed);
        long IError = 0;
        setAllDrivePower(vy,vy,-vy,-vy);
        int previousPos = offsetY, currentOdometry, Dterm;
        double tpre = 0, tcur;

        while(!this.gamepad1.b){
            currentOdometry = getYOdometry();
            tcur=t.milliseconds();
            Dterm = (int)((currentOdometry - previousPos)/(tcur-tpre));
            IError += (currentOdometry - odometryYGoal)*(tcur-tpre);
            multiply_factor = -Math.min(1, Math.max(-(1/speed), ((params[0] * (currentOdometry - odometryYGoal)/odometryEncPerInch) +  (near(Dterm,0,speed * 5000 / 0.3)?(params[2] * Dterm):0)) + (params[1] * IError )));
            previousPos = currentOdometry;
            tpre=tcur;

            setAllDrivePowerG(multiply_factor*-vy,multiply_factor*-vy,multiply_factor*vy,multiply_factor*vy, params[5]);

            writeLog(t.milliseconds()+", "+currentOdometry+", "+((currentOdometry - odometryYGoal)/odometryEncPerInch)+", "+((currentOdometry - odometryYGoal)/odometryEncPerInch)*params[0]+", "+IError+", "+Dterm+", "+(near(Dterm,0,speed * 5000 / 0.3)?(params[2] * Dterm):"CLIPPED")+", "+LF.getPower()+", "+LF.getCurrentPosition()+", "+getHeading());
        }
        setAllDrivePower(0);
        writeLogHeader("Gyro drift="+getHeading()+", Xdrift="+getXOdometry());
        writeLogHeader("----End of run----");
    }
}
