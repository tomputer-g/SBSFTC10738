package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode20.BaseAuto;
import org.openftc.revextensions2.RevBulkData;

public class DrivetrainThreadTest extends BaseAuto {
    DrivetrainThread drivetrainThread;


    @Override
    public void runOpMode() throws InterruptedException {
        drivetrainThread = new DrivetrainThread();
        initOdometry();
        waitForStart();
        while(opModeIsActive()){

        }
    }

    class DrivetrainThread extends Thread{
        boolean stop = false;
        double[] odometryPositions = {0,0,0};//X (xOdo), Y1 (platform), Y2 (L2)
        double[] odometrySpeeds = {0,0,0};


        double[] lastPositions = {0,0,0};//X, Y, Theta; TODO: what about the 3rd odo wheel?
        double[] lastSpeeds = {0,0,0}; //vX, vY, Omega
        double[] lastAccelerations = {0,0,0}; // ax, ay, alpha
        long lastTimestampNS = 0;
        ElapsedTime t;
        @Override
        public void run() {
            t = new ElapsedTime();
            while(!stop && !isInterrupted()){
                //keeping track of last values
                update();
            }
        }

        public void update(){
            //update timestamp & get dT
            long now = t.nanoseconds();
            double deltaTimeS = (now - lastTimestampNS) / 1.0E9;
            lastTimestampNS = now;

            //reading current positions & theta
            //note: Bulk reading is 4.3x faster; this makes reading both odo. encoders at the same time worth it.

            decomposeOdometryWheels(deltaTimeS);
            double[] currentPositions = {0,0,getHeading()};//TODO: account for IMU resets & -179 -> 179
            double[] currentSpeeds = {0,0,0};
            for(int i = 0;i < 3; i++){
                currentSpeeds[i] = (currentPositions[i] - lastPositions[i]) / deltaTimeS;//setting current speeds
                lastAccelerations[i] = (currentSpeeds[i] - lastSpeeds[i]) / deltaTimeS;//setting accelerations
            }
            lastPositions = currentPositions;
            lastSpeeds = currentSpeeds;
        }

        public void decomposeOdometryWheels(double deltaTime){
            RevBulkData bulkOdo = hub4.getBulkInputData();
            double[] currentOdoPos = {bulkOdo.getMotorCurrentPosition(xOdometry), bulkOdo.getMotorCurrentPosition(platform_grabber), bulkOdo.getMotorCurrentPosition(L2)};
            for (int i = 0;i < 3;i++){
                odometrySpeeds[i] = (currentOdoPos[i] - odometryPositions[i])/deltaTime;
            }
            //input: 3 odometry velocities
            //output: X,Y,R components of each wheel -> average into three current speeds
            double[] speedComponents = {0,0,0};
            double[] coeffs = {0,0,0};//X R coeff; Y(plat) R coeff; Y(L2) R coeff;


            //wheel 2 & 3: Y odometry (platform, L2)
            //2 equations for 2 var.s (Y, R)
            //Vplatform = Y + cR
            //VL2 = (+-?)Y + cR
            //TODO: find polarity & calculate


            //wheel 1: X odometry
            //equation with R known
            speedComponents[0] = odometrySpeeds[0] - coeffs[0] * speedComponents[2]; // X component = odoX - c*R

        }

    }
}
