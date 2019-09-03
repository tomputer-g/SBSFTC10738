package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BaseAuto;

import java.util.List;
@Disabled
@Autonomous
public class TFOD_and_VuMark_transition_test extends BaseAuto {
    private boolean autoDone = false;
    @Override
    public void init() {
        initVuforiaEngine();
        initTFOD();
        tfod.activate();
    }

    @Override
    public void init_loop() {
        List<Recognition> recs = tfod.getRecognitions();
        for(Recognition r : recs){
            telemetry.addData(r.getLabel(),"("+r.getLeft()+", "+r.getBottom()+")");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        if(!autoDone) {
            tfod.deactivate();
            tfod.shutdown();
            wait(200);
            initVumark();
            telemetry.addLine("Vumark init");
            telemetry.update();
            autoDone = true;
        }
    }
}
