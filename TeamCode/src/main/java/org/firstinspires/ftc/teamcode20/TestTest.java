package org.firstinspires.ftc.teamcode20;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class TestTest extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        double time =0;
        boolean[] st={true};
        initIMU();
        initDrivetrain();
        initOdometry();
        initHubs();
        waitForStart();
        ElapsedTime t = new ElapsedTime();
        while(!this.gamepad1.b){
            if(zheng(this.gamepad1.start,st)){
                resetY1Odometry();
                resetY2Odometry();
            }
            telemetry.addData("Y1",getY1Odometry());
            telemetry.addData("Y2",getY2Odometry());
            telemetry.update();
        }
        //setAllDrivePower(-.3,-.3,.3,.3);

    }
}
