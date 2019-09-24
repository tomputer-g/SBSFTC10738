package org.firstinspires.ftc.teamcode19.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode19.BaseAuto;
import org.firstinspires.ftc.teamcode19.BaseOpMode;

@TeleOp
public class AutoMoveInchTest extends BaseOpMode {
    ElapsedTime t;
    private DistanceSensor sensorRange1,sensorRange2;
    @Override public void init() {
        initDrivetrain();
        sensorRange1=hardwareMap.get(DistanceSensor.class, "sensor_range1");
        sensorRange2=hardwareMap.get(DistanceSensor.class, "sensor_range2");
        t = new ElapsedTime();
        double dist1, dist2;
    }

    @Override
    public void loop() {
        dist1 = sensorRange1.getDistance(DistanceUnit.METER);
        dist2 = sensorRange2.getDistance(DistanceUnit.METER);
        telemetry.addData("range1", String.format("%.01f m", dist1));
        telemetry.addData("range2", String.format("%.01f m", dist2));

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
            
        }
    }
}
