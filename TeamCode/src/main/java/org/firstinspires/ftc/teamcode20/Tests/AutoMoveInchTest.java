package org.firstinspires.ftc.teamcode20.Tests;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode19.BaseOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoMoveInchTest extends BaseOpMode {
    ElapsedTime t;
    //private DistanceSensor sensorRange1,sensorRange2;
    private ModernRoboticsI2cRangeSensor ultra1;
    double dist1, dist2;
    @Override public void init() {
        initDrivetrain();
        /*
        sensorRange1=hardwareMap.get(DistanceSensor.class, "sensor_range1");
        sensorRange2=hardwareMap.get(DistanceSensor.class, "sensor_range2");
        */
        ultra1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra1");
       // ultra2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra2");

        t = new ElapsedTime();
    }

    @Override
    public void loop() {
        /*
        dist1 = sensorRange1.getDistance(DistanceUnit.INCH);
        distSide = sensorRange2.getDistance(DistanceUnit.INCH);
        */
        dist1 = ultra1.getDistance(DistanceUnit.INCH);
       // distSide = ultra2.getDistance(DistanceUnit.INCH);
        /*
        telemetry.addData("range1", String.format("%.01f inch", dist1));
        telemetry.addData("range2", String.format("%.01f inch", distSide));
*/
        //telemetry.addData("distUltra", String.format(".2f cm", dist_ultra));
        telemetry.addData("dist1:", "%.2f inch", dist1);
        telemetry.addData("distSide:", "%.2f inch", dist2);

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
            /*
            while(this.gamepad1.left_bumper);
            double dist;
           // setAllDrivePower(0.1,0.1,0.1,0.1);
            //wait(3000);
            //setAllDrivePower(0);
            for(int xx = 0;xx<=1;++xx){
                while(Math.abs(dist1-distSide)>0.2){
                    dist = dist1-distSide;
                    telemetry.addData("remaining dist: ",dist + "inch");

                    //magic number 0.00347
                    if(dist > 0) setAllDrivePower(Math.min(0.00347*dist*dist, 0.4));
                    else setAllDrivePower(Math.max(-0.00347*dist*dist, -0.4));

                    dist1 = ultra1.getDistance(DistanceUnit.INCH);
                    distSide = ultra2.getDistance(DistanceUnit.INCH);
                    telemetry.addData("range1", String.format("%.01f inch", dist1));
                    telemetry.addData("range2", String.format("%.01f inch", distSide));
                    telemetry.update();
                }
                setAllDrivePower(0);
            }
            setAllDrivePower(0);
             */
        }
    }
}
