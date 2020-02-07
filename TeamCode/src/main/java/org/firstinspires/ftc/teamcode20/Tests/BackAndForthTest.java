package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;
@TeleOp
public class BackAndForthTest extends BaseAuto {
    boolean d[] = {true};
    int ops;
    public void init() {
        super.init();
        initAutonomous();
        ops = 0;
    }

    @Override
    public void loop() {
        if(zheng(this.gamepad1.a,d)){
            for(int i = 0;i<3;++i){
                moveInchesGOY_XF_F(-88,0.6,1,ops);
                moveInchesGOY_XF_F(+88,0.6,1,ops);
            }
        }
    }
}
