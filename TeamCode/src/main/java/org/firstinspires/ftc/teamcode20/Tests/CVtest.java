package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode20.BaseAuto;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import java.util.Locale;

@TeleOp
public class CVtest extends BaseAuto {
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;
    @Override
    public void init(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);
    }
    @Override
    public void start(){
        phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
    }
    @Override
    public void loop(){
        telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
        telemetry.addData("Frame Count", phoneCam.getFrameCount());
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
        telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
        telemetry.update();
    }
    @Override
    public void stop(){
        phoneCam.stopStreaming();
    }
}
