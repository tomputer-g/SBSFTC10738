package org.firstinspires.ftc.teamcode20.Roadrunner.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class OdometryWheelLocalizer extends ThreeTrackingWheelLocalizer {
    private ExpansionHubEx hub4;
    private DcMotorEx L2, xOdo, platform;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed


    public OdometryWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(//x and y are reversed
                new Pose2d(48/25.4, -166.15/25.4,  Math.toRadians(180)),
                new Pose2d(-100/25.4, 4/25.4,     Math.toRadians(270)),//TODO: confirm?
                new Pose2d(48/25.4, 166.15/25.4,   Math.toRadians(180))));
        L2 = hardwareMap.get(DcMotorEx.class,"L2");
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        platform = hardwareMap.get(DcMotorEx.class, "platform");
        platform.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platform.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xOdo = hardwareMap.get(DcMotorEx.class,"xOdo");
        xOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hub4 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");
    }


    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NotNull @Override public List<Double> getWheelPositions() {
        RevBulkData bulk = hub4.getBulkInputData();
        return Arrays.asList(
                encoderTicksToInches(bulk.getMotorCurrentPosition(L2)),
                encoderTicksToInches(bulk.getMotorCurrentPosition(xOdo)),
                encoderTicksToInches(bulk.getMotorCurrentPosition(platform))
        );
    }
}
