package org.firstinspires.ftc.teamcode20.Tests;

import org.firstinspires.ftc.teamcode20.BaseOpMode;

public class LimitAccelerationMoveTest extends BaseOpMode {
    private final double dAPerSec = 0.8f;//TODO: what's the actual thing?

    @Override
    public void init() {
        initDrivetrain();
    }

    @Override
    public void loop() {
        super.loop();
    }

    private void noSkidMoveInches(double dX, double dY, double maxV){

    }

    private void noSkidTurn(double dTheta){

    }

    private void noSkidTeleOpMove(double vX, double vY, double vR){//kind of a Proportional system where we 'follow' the joystick but a const rate of power change

    }

}
