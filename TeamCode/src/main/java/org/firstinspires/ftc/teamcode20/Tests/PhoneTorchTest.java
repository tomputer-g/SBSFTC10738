package org.firstinspires.ftc.teamcode20.Tests;

import android.content.Context;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraManager;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static java.lang.Thread.sleep;

public class PhoneTorchTest extends OpMode {
    CameraManager cameraManager;
    @Override
    public void init() {
         cameraManager = (CameraManager) hardwareMap.appContext.getSystemService(Context.CAMERA_SERVICE);
         String[] cameraIDs = {};
        try {
            cameraIDs = cameraManager.getCameraIdList();
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
        for(String s : cameraIDs){
            telemetry.addData("ID", s);

        }
        telemetry.update();
        //cameraIDs


    }

    @Override
    public void loop() {
        try {
            cameraManager.setTorchMode("0", true);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
        try {
            sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        try {
            cameraManager.setTorchMode("0", false);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
        try {
            sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
}
