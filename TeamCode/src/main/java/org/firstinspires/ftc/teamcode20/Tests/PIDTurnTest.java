
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
    private double kP=0.088,kD=0;
    private int magnitude=-1;
    private boolean[] du ={true}, dd={true}, dl={true},dr={true},rb={true},y={true},a={true},x={true},b={true},lb={true};
    private double imuinitvalue=0, target=90;
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
        if(zheng(this.gamepad1.left_bumper,lb)){
            PIDturn(target,kD,kP,0.3);
            setNewGyro0();
        }
    }

    public void PIDturn(double target, double kd, double kp,double speed){
        setNewGyro0();
        double e = target;
        ElapsedTime t = new ElapsedTime();
        speed=(target>0)?speed:-speed;
        while(!this.gamepad1.right_bumper){
            double e2 = target-(getHeading());
            double D = kd*(e2-e)/t.milliseconds();
            double P = e2*kp;
            if(Math.abs(P)>Math.abs(speed))P=speed;
            setAllDrivePower(P+D,P+D,P+D,P+D);
            e=e2;
            t.reset();
        }
        setAllDrivePower(0.0);

    }
}
