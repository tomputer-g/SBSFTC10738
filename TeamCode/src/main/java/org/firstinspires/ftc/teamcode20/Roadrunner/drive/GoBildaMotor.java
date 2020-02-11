package org.firstinspires.ftc.teamcode20.Roadrunner.drive;


import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev = 145.6, gearing = 5.2, maxRPM = 1150, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "goBILDA5202SeriesMotor", name="GoBILDA 5202 1150RPM")
@DistributorInfo(distributor = "goBILDA_distributor", model = "goBILDA-5202-0005", url="https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motors/")

public interface GoBildaMotor {
}
