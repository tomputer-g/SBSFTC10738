package org.firstinspires.ftc.teamcode20.Roadrunner.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class OdometryWheelLocalizer extends ThreeTrackingWheelLocalizer {
    private ExpansionHubEx hub4;


    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed


    public OdometryWheelLocalizer(HardwareMap hardwareMap) {

        //1: L2
        //2: xodo
        //3: platform
        //y: 44 forward, 149 side
        //x: 100 back, 4 left (mms)


        hub4 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");
    }


    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NotNull @Override public List<Double> getWheelPositions() {
        RevBulkData bulk = hub4.getBulkInputData();
        return Arrays.asList(
                encoderTicksToInches(bulk.getMotorCurrentPosition(1)),
                encoderTicksToInches(bulk.getMotorCurrentPosition(3)),
                encoderTicksToInches(bulk.getMotorCurrentPosition(2))
        );
    }
}
