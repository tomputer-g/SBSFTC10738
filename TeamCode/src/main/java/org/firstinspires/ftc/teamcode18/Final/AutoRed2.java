package org.firstinspires.ftc.teamcode18.Final;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode18.BaseAuto;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Ziming Gao on 12/5/2017.
 */
@Disabled
@Autonomous(name = "Red_2", group = "Final")
public class AutoRed2 extends BaseAuto {
    @Override
    public void loop(){
        if(!autonomousDone){
            doJewel("Red");
            for(int i = 0;i < 10; i++){
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {finalVuMark = vuMark;}
            }//VuMark ID
            turn(-15);
            moveRightDistance(1050,autonomousRotSpeed);
            int distance = 800;
            if(finalVuMark != null){
                switch(finalVuMark){
                    case LEFT:
                        distance = 1100;
                        break;
                    case CENTER:
                        distance = 800;
                        break;
                    case RIGHT:
                        distance = 450;
                        break;
                }
            }

            moveForwardDistance(distance,0.2);
            turn(-110);
            grabber.setTargetPosition(0);
            moveForwardDistance(600,0.1);
            autonomousDone = true;
        }
    }
}
