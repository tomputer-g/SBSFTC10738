package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp
public class BackAndForthTest extends BaseAuto {

    private double[] params = {1,0};
    private String[] paramNames = {"kT","ops"};
    private int currentSelectParamIndex = 0;
    private boolean a, l, r, u, d, lb, rb;

    public void init() {
        super.init();
        initAutonomous();
        initViewMarks();
        //initVuMarksFull();
        //kuaishou.comkuaishou.comkuaishou.comkuaishou.comkuaishou.comkuaishou.comkuaishou.comkuaishou.comkuaishou.comkuaishou.com
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){a = true;}if(!this.gamepad1.a && a){
            servoThread.setTarget(0.95);
            platform_grabber.setPower(1);
            platform_grabber.setPower(0.0);
            if(showTelemetry)telemetry.clear();
            grabber.setPosition(grabber_open);
            a = false;
            params[1] = getXOdometry();
            double origin[] = {0,41}, dd[]=adjustToViewMark(true), realPOS[] = {0,0};
            //telemetry.addData("posX","%.2f" ,origin[0]);
            telemetry.addData("posY", "%.2f",origin[1]);
            telemetry.update();
            for(int i = 0;i<4;++i){
                align(0);
                moveInchesGOY_XF_F(-71.75-8*i,0.6,1,(int) (-(origin[1]-dd[1])*odometryEncXPerInch));
                //realPOS = vuMarkPos();

                //telemetry.addData("x",realPOS[0]);
                //telemetry.addData("y",realPOS[1]);
                telemetry.addData("xodo",getXOdometry());
                align(90);
                setNewGyro(-90);
                moveInchesGOY(6,0.4,1);
                grabber.setPosition(grabber_closed);
                wait(300);
                servoThread.setTarget(0.85);
                //setAllDrivePower(0.0);
                moveInchesG(0,-6,0.4);
                PIDturnfast(90,false);
                setNewGyro(0);
                //telemetry.addData("x1",realPOS[0]);
                //telemetry.addData("y2",realPOS[1]);
                telemetry.addData("xodo",getXOdometry());

                telemetry.update();
                wait(500);
                moveInchesGOY_XF_F(i*8+71.75,0.6,1,(int) (-(origin[1]-dd[1])*odometryEncXPerInch));
                dd=adjustToViewMark(true);
                //telemetry.addData("posX","%.2f" ,dd[0]);
                telemetry.addData("original", "%.2f",origin[1]);
                telemetry.addData("current", "%.2f",dd[1]);
                telemetry.update();
            }
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

        //telemetry.addData("parameters",params[0]+", "+params[1]);
        //telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
        //telemetry.update();
        //telemetry.addData("x",getXOdometry());
    }

}
