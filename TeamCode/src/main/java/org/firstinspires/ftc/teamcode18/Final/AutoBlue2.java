package org.firstinspires.ftc.teamcode18.Final;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode18.BaseAuto;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Ziming Gao on 12/5/2017.
 */
@Disabled
@Autonomous(name = "Blue_2", group = "Final")
public class AutoBlue2 extends BaseAuto {
    @Override
    public void loop(){
        if(!autonomousDone){
            doJewel("Blue");
            for(int i = 0;i < 10; i++){
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {finalVuMark = vuMark;}
            }//VuMark ID
            turn(-15);
            moveLeftDistance(1200,autonomousRotSpeed);
            int distance = 0;
            switch(finalVuMark){
                case LEFT:
                    distance = 410;
                    break;
                case CENTER:
                    distance = 740;
                    break;
                case RIGHT:
                    distance = 1100;
                    break;
                default:
                    distance = 740;
                    break;
            }
            moveForwardDistance(distance,0.2);
            turn(110);
            grabber.setTargetPosition(0);
            moveForwardDistance(400,0.1);
            autonomousDone = true;
        }
    }
}
