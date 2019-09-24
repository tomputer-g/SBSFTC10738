package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode19.BaseOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode19.BaseAuto;
import org.firstinspires.ftc.teamcode19.BaseOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class AutoMoveInchTest extends BaseOpMode {
    ElapsedTime t;
    private DistanceSensor sensorRange1,sensorRange2;
    double dist1, dist2;
    @Override public void init() {
        initDrivetrain();
        sensorRange1=hardwareMap.get(DistanceSensor.class, "sensor_range1");
        sensorRange2=hardwareMap.get(DistanceSensor.class, "sensor_range2");
        t = new ElapsedTime();
    }

    @Override
    public void loop() {
        dist1 = sensorRange1.getDistance(DistanceUnit.INCH);
        dist2 = sensorRange2.getDistance(DistanceUnit.INCH);
<<<<<<< Updated upstream
        telemetry.addData("range1", String.format("%.01f inch", dist1));
        telemetry.addData("range2", String.format("%.01f inch", dist2));
=======
        telemetry.addData("range1", String.format("%.01f m", dist1));
        telemetry.addData("range2", String.format("%.01f m", dist2));
>>>>>>> Stashed changes

        if(this.gamepad1.a){
            while(this.gamepad1.a);
            t.reset();
            moveInches(30,0,1);
            telemetry.addData("Time",t.milliseconds()+"ms");
            telemetry.update();
            //moveInchesHighSpeedEncoder(30,0,1,5,5,0.2,0.2,0);
        }else if(this.gamepad1.y){
            while(this.gamepad1.y);
            t.reset();
            moveInches(0,30,1);
            telemetry.addData("Time",t.milliseconds()+"ms");
            telemetry.update();
            //moveInchesHighSpeedEncoder(0,30,1,5,5,0.2,0.2,0);
        }else if(this.gamepad1.x){
            while(this.gamepad1.x);
            t.reset();
            moveInches(-30,0,1);
            telemetry.addData("Time",t.milliseconds()+"ms");
            telemetry.update();
            //moveInchesHighSpeedEncoder(-30,0,1,5,5,0.2,0.2,0);
        }else if(this.gamepad1.b) {
            while (this.gamepad1.b);
            t.reset();
            moveInches(0, -30, 1);
            telemetry.addData("Time",t.milliseconds()+"ms");
            telemetry.update();
            //moveInchesHighSpeedEncoder(0,-30,1,5,5,0.2,0.2,0);
        }else if(this.gamepad1.left_bumper){
            while(this.gamepad1.left_bumper);
            double dist;
            while(Math.abs(dist1-dist2)<0.5){
                dist = Math.abs(dist1-dist2);   
                telemetry.addData("aaa",dist + "inch");
                setAllDrivePower(-dist,-dist,dist,dist);
                dist1 = sensorRange1.getDistance(DistanceUnit.INCH);
                dist2 = sensorRange2.getDistance(DistanceUnit.INCH);
            }
            setAllDrivePower(0);
        }
    }
}
