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
    }

    @Override
    public void loop() {
        if(this.gamepad1.a){a = true;}if(!this.gamepad1.a && a){
            a = false;
            params[1] = getXOdometry();
            for(int i = 0;i<3;++i){
                moveInchesGOY_XF_F(-88,0.6,params[0],(int)params[1]);
                moveInchesGOY_XF_F(+88,0.6,params[0],(int)params[1]);
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

        telemetry.addData("parameters",params[0]+", "+params[1]);
        telemetry.addData("now changing", paramNames[currentSelectParamIndex]);
        telemetry.update();
    }

}
