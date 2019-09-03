package org.firstinspires.ftc.teamcode18.Final;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode18.BaseAuto;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Ziming Gao on 12/5/2017.
 */
@Disabled
@Autonomous(name = "Red_1", group = "Final")
public class AutoRed1 extends BaseAuto {
    @Override
    public void loop(){
        if(!autonomousDone){
            doJewel("Red");
            for(int i = 0;i < 10; i++){
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {finalVuMark = vuMark;}
            }//VuMark ID
            turn(-15);
            int distance = 0;
            switch(finalVuMark){
                case LEFT:
                    distance = 1600;
                    break;
                case CENTER:
                    distance = 1300;
                    break;
                case RIGHT:
                    distance = 875;
                    break;
                default:
                    distance = 1300;
                    break;
            }
            moveRightDistance(distance,0.2);
            turn(-160);
            grabber.setTargetPosition(0);
            moveForwardDistance(700,0.2);
            autonomousDone = true;
        }
    }
}
