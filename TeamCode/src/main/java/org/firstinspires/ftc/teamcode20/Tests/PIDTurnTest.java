
package org.firstinspires.ftc.teamcode20.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode20.BaseOpMode;

@TeleOp

public class PIDTurnTest extends BaseAuto {
    private double kP=0.068,kD=0.9;
    private int magnitude=-2;
    private boolean[] du ={true}, dd={true}, dl={true},dr={true},rb={true},y={true},a={true},x={true},b={true},lb={true};
    private double imuinitvalue=0, target=90, result=0;
    @Override
    public void init() {
        initDrivetrain();
        initIMU();
        initOdometry();
        wait(200);
        setNewGyro0();
    }
    @Override
    public void loop() {
        if(zheng(this.gamepad1.dpad_up,du)){kP+=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.dpad_down,dd)){kP-=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.y,y)){kD+=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.a,a)){kD-=Math.pow(10,magnitude);}
        if(zheng(this.gamepad1.x,x)){target+=5;}
        if(zheng(this.gamepad1.b,b)){target-=5;}
        if(zheng(this.gamepad1.dpad_left,dl)){magnitude++;}
        if(zheng(this.gamepad1.dpad_right,dr)){magnitude--;}
        telemetry.addData("magnitude: ",Math.pow(10,magnitude));
        telemetry.addData("kP: ",kP);
        telemetry.addData("kD: ",kD);
        telemetry.addData("imu: ",getHeading());
        telemetry.addData("target:",target);
        telemetry.addData("result: ",result);
        telemetry.addLine("LF: "+LF.getCurrentPosition()+" LB: "+LB.getCurrentPosition()+" RF: "+RF.getCurrentPosition()+" RB:"+RB.getCurrentPosition());
        if(zheng(this.gamepad1.left_bumper,lb)){
            PIDturn(target,kD,kP,0.5);
            setNewGyro0();
        }
    }

    public void PIDturn(double target, double kd, double kp,double speed){
        setNewGyro0();
        double e = target;
        ElapsedTime t = new ElapsedTime();
        while(!this.gamepad1.right_bumper){
            double e2 = target-(getAdjustedHeading(target));
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(speed))P=P>0?speed:-speed;
            setAllDrivePower(P+D,P+D,P+D,P+D);
            e=e2;
            t.reset();
        }
        setAllDrivePower(0.0);
        result=getHeading();

    }

    private double getAdjustedHeading(double target){
        double i = getHeading();
        if(target>0)
            return i<0?i+360:i;
        else
            return i>0?i-360:i;
    }
}
