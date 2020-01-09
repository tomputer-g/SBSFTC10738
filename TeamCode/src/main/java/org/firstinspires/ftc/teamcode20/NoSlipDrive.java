package org.firstinspires.ftc.teamcode20;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp

public class NoSlipDrive extends BaseAuto {
    ElapsedTime t;
    final double ymult = 232.7551/12;
    final double odomult = 4096/Math.PI;
    //button press booleans
    boolean[] rb={true},lb={true};

    //previous motor counts
    int lfmc=0,lbmc=0,rfmc=0,rbmc=0;

    //delta motor counts
    double lfdc,lbdc,rfdc,rbdc;

    //delta odometry count, previoud odometry count
    double odc=0; int omc;

    @Override
    public void init() {
        telemetryOn = true;
        initDrivetrain();
        initPlatformGrabber();
        initIMU();
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(1);
        wait(150);
        platform_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platform_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform_grabber.setPower(0);
        wait(500);
        t = new ElapsedTime();
        platform_grabber.setPower(1);
        wait(300);
        platform_grabber.setPower(0.0);
    }

    @Override
    public void loop() {
        t.reset();
        //scaledMove(-this.gamepad1.left_stick_x * 0.3, -this.gamepad1.left_stick_y * 0.15, (this.gamepad1.left_bumper ? 0 : -this.gamepad1.right_stick_x * 0.2));
        if(zheng(this.gamepad1.right_bumper,rb)){
            platform_grabber.setPower(-.8);
            wait(200);
        }
        if(zheng(this.gamepad1.left_bumper,lb)){
            for(int i=0;i<1000000;i++) {
                noslippower(-0.5, -0.5, 0.5, 0.5);
                updateMC();
                updateOC();
                wait(20);
            }
        }
    }

    protected void noslippower(double lf,double lb, double rf, double rb){
        setAllDrivePowerG(lfdc>odc?lf:lf/2,lbdc>odc?lb:lb/2,rfdc>odc?rf:rf/2,rbdc>odc?rb:rb/2);
    }

    private void updateMC(){
        //update the delta MC
        lfdc=(double)(LF.getCurrentPosition()-lfmc)/ymult;lbdc=(double)(LB.getCurrentPosition()-lbmc)/ymult;rfdc=(double)(RF.getCurrentPosition()-rfmc)/ymult;rbdc=(double)(RB.getCurrentPosition()-rbmc)/ymult;
        //update the previous MC
        lfmc=LF.getCurrentPosition();lbmc=LB.getCurrentPosition();rfmc=RF.getCurrentPosition();rbmc=RB.getCurrentPosition();
    }

    private void updateOC(){
        odc=(double)(L2.getCurrentPosition()-omc)/odomult);
        omc=L2.getCurrentPosition();
    }


}
