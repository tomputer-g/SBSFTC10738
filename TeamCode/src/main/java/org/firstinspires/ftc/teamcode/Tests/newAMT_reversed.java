package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
created by Lucien Liu in Dec 2018
 */
import org.firstinspires.ftc.teamcode.BaseAuto;
@Autonomous
@Disabled
public class newAMT_reversed extends BaseAuto {

    @Override
    public void loop() {
        myMove(1,0.3);
        requestOpModeStop();
    }

    public void myMove(double desired_speed, double time_travelled) {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double acc_rate = 0.05;
        double dec_rate = 0.94;
        double tolerance = 0.002;

        //accelerating part
        for(double i = 0;i<=desired_speed;i+=acc_rate) {
            LF.setPower(i);
            LB.setPower(i);
            RF.setPower(-i);
            RB.setPower(-i);
        }

        ElapsedTime t = new ElapsedTime();
        t.reset();

        //moving with desired speed
        while(t.milliseconds() < time_travelled*1000);

        //decelerating part I
        for(double i = desired_speed;i>tolerance;i*=dec_rate) {
            LF.setPower(i);
            LB.setPower(i);
            RF.setPower(-i);
            RB.setPower(-i);
        }

        //waiting for 1 sec
        t.reset();
        while(t.milliseconds() < 2000)
        {
            //decelerating part II
            LF.setPower(-0.05);
            LB.setPower(-0.05);
            RF.setPower(0.05);
            RB.setPower(0.05);
        }

        //stop
        setAllDrivePower(0);
    }
}
