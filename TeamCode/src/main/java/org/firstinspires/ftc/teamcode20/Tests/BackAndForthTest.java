package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp
public class BackAndForthTest extends BaseAuto {

    private double[] params = {1,0};
    private String[] paramNames = {"kT","ops"};
    private int currentSelectParamIndex = 0;
    private boolean a, l, r, u, d, lb, rb;
    private double speed = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();
        initViewMarks();
        //initVuMarksFull();
        waitForStart();
        while(opModeIsActive())
        {
            if(this.gamepad1.a){a = true;}if(!this.gamepad1.a && a){
            a = false;

            double curX;
            //params[1] = getXOdometry();
            double origin[] = {0,41}, dd[]=adjustToViewMark(true);
            //telemetry.addData("posX","%.2f" ,origin[0]);
            telemetry.addData("posY", "%.2f",origin[1]);
            telemetry.update();
            for(int i = 0;i<2;++i){
                setAllDrivePower(0);
                curX = getXOdometry();
                if(i>0)servoThread.setTarget(0.75);
                grabber.setPosition(grabber_open);
                align(0);
                moveInchesGOY_XF_F(-74.75-24*i,0.6,1,(int) (curX-(origin[1]-dd[1])*odometryEncXPerInch));
                servoThread.setTarget(0.98);
                align(-90);

                double yorigin = getY1Odometry();
                while((getY1Odometry()-yorigin)*-1 < odometryEncYPerInch*4){
                    setAllDrivePowerG(-.3,-.3,.3,.3);
                }
                while((getY1Odometry()-yorigin)*-1 < odometryEncYPerInch*8){
                    setAllDrivePowerG(-speed,-speed,speed,speed);
                }
                grabber.setPosition(grabber_closed);
                wait(300);
                servoThread.setTarget(0.85);
                while((getY1Odometry()-yorigin)*-1 > odometryEncYPerInch*2){
                    setAllDrivePowerG(.3,.3,-.3,-.3);
                }
                setAllDrivePower(0);
                align(0);
                servoThread.setTarget(0.6);
                moveInchesGOY_XF_F(24*i+74,0.6,1,(int) (curX-(origin[1]-dd[1])*odometryEncXPerInch));
                dd=adjustToViewMark(true);
                //telemetry.addData("original", "%.2f",origin[1]);
                //telemetry.addData("current", "%.2f",dd[1]);
                //telemetry.update();
            }
        }
            if(this.gamepad1.left_bumper){lb = true;}if(!this.gamepad1.left_bumper && lb){
            lb = false;
            speed-=0.01;
            /*
            currentSelectParamIndex--;
            if(currentSelectParamIndex < 0){
                currentSelectParamIndex = params.length - 1;
            }

             */
        }
            if(this.gamepad1.right_bumper){rb = true;}if(!this.gamepad1.right_bumper && rb){
            rb = false;
            speed+=0.01;
            /*
            currentSelectParamIndex++;
            if(currentSelectParamIndex >= params.length){
                currentSelectParamIndex = 0;
            }

             */
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
            telemetry.addData("s,",speed);
            //telemetry.addData("parameters",params[0]+", "+params[1]);
            //telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
            telemetry.update();
            //telemetry.addData("x",getXOdometry());
        }
    }


}
