package org.firstinspires.ftc.teamcode20;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp

public class NoSlipDrive extends BaseAuto {
    ElapsedTime t;
    final double ymult = 232.7551/12;
    final double odomult = 4096/Math.PI;
    int phase =0;
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
        initDrivetrain();
        initPlatformGrabber();
        initIMU();
        initOdometry();
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
            phase = 1;
        }
        telemetry.addLine("Odo: "+L2.getCurrentPosition());
        telemetry.addLine("LF: "+LF.getCurrentPosition());
        telemetry.addLine("LB: "+LB.getCurrentPosition());
        telemetry.addLine("RF: "+RF.getCurrentPosition());
        telemetry.addLine("RB: "+RB.getCurrentPosition());
        if(zheng(this.gamepad1.left_bumper,lb)){
            t = new ElapsedTime();
            for(int i=0;i<1000000;i++) {
                if(phase==1)noslippower(-0.5, -0.5, 0.5, 0.5);
                if(true){
                    updateMC();
                    updateOC();
                    telemetry.addLine("dLF: "+lfdc+" dRF: "+rfdc);
                    telemetry.addLine("odc: "+odc);
                    telemetry.update();
                }
            }
        }
    }

    protected void noslippower(double lf,double lb, double rf, double rb){
        setAllDrivePower(lfdc>=odc?0.1:lf,lbdc>=odc?0.1:lb,rfdc>=odc?0.1:rf,rbdc>=odc?0.1:rb);
    }

    private void updateMC(){
        //update the delta MC
        lfdc=(double)(-lfmc+LF.getCurrentPosition())/ymult;lbdc=(double)(-lbmc+LB.getCurrentPosition())/ymult;rfdc=(double)(-RF.getCurrentPosition()+rfmc)/ymult;rbdc=(double)(-RB.getCurrentPosition()+rbmc)/ymult;
        //update the previous MC
        lfmc=LF.getCurrentPosition();lbmc=LB.getCurrentPosition();rfmc=RF.getCurrentPosition();rbmc=RB.getCurrentPosition();
    }

    private void updateOC(){
            odc = (double) (L2.getCurrentPosition() - omc) / odomult;
            omc = L2.getCurrentPosition();
    }


}
