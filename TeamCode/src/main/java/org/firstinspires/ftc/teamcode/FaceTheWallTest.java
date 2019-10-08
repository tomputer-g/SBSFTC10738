package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FaceTheWallTest extends BaseAuto {
    ModernRoboticsI2cRangeSensor rangeSensor1, rangeSensor2;
    ElapsedTime t;
    double dist1, dist2;
    @Override
    public void init() {
        initDrivetrain();
        t = new ElapsedTime();
        rangeSensor1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "s1");
        rangeSensor2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "s2");
    }

    @Override
    public void loop() {
        if(this.gamepad1.start){
            while(this.gamepad1.start);
            dist1 = rangeSensor1.getDistance(DistanceUnit.INCH);
            dist2 = rangeSensor2.getDistance(DistanceUnit.INCH);
            while(!(near(dist1,12,0.3) && near(dist2,12,0.3))){
                setAllDrivePower(0.5,0);
            }
            while(!near(,90)){
                setAllDrivePower(0.5);
            }
        }
    }
}
