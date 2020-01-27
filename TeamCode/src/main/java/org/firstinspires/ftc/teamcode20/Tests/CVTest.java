package org.firstinspires.ftc.teamcode20.Tests;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.Locale;

@TeleOp
public class CVTest extends BaseAuto {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;
    private StoneDetector stoneDetector;
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        //skyStoneDetector = new SkystoneDetector();
        stoneDetector=new StoneDetector();
        stoneDetector.stonesToFind=1;
        phoneCam.setPipeline(stoneDetector);
    }
    @Override
    public void start(){
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }
    @Override
    public void loop(){
        int n=stoneDetector.foundScreenPositions().size();
        telemetry.addData("Num Stones:",n);
        try {
            telemetry.addData("Stone Position X", stoneDetector.foundScreenPositions().get(0).x);
            telemetry.addData("Stone Position Y", stoneDetector.foundScreenPositions().get(0).y);
            telemetry.addLine("X: "+ stoneDetector.foundRectangles().get(0).x+" Y: "+stoneDetector.foundRectangles().get(0).y);
            //telemetry.addLine()
        }
        catch(Exception e){}
        //telemetry.addData("Frame Count", phoneCam.getFrameCount());
        //telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
        //telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        //telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        //telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        //telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
        telemetry.update();
    }
    @Override
    public void stop(){
        phoneCam.stopStreaming();
    }
}
