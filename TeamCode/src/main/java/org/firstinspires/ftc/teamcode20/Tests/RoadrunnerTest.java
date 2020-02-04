package org.firstinspires.ftc.teamcode20.Tests;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

import org.firstinspires.ftc.teamcode20.BaseAuto;

public class RoadrunnerTest extends BaseAuto {
    @Override
    public void init() {
        PIDCoefficients coeff = new PIDCoefficients(0,0,0);
        PIDFController controller = new PIDFController(coeff);

    }

    @Override
    public void loop() {

    }
}
