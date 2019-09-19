package org.firstinspires.ftc.teamcode18.Listeners;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * Created by Ziming Gao on 12/5/2017.
 */
public class GRVListener implements SensorEventListener {
    public double roll, deriv = 0;
    private double lastvalue;
    private int i = 1;
    private float[] mRotationMatrix = new float[16];
    public GRVListener() {

            mRotationMatrix[0] = 1;
            mRotationMatrix[4] = 1;
            mRotationMatrix[8] = 1;
            mRotationMatrix[12] = 1;
        }
    @Override public void onSensorChanged(SensorEvent sensorEvent) {
        SensorManager.getRotationMatrixFromVector(mRotationMatrix, sensorEvent.values);
        roll = Math.toDegrees(Math.asin((double)mRotationMatrix[0]));
        if(i == 1)
            lastvalue = roll;
        if(i == 5) {
            deriv = lastvalue - roll;
            i = 0;
        }
        i++;
    }
    @Override public void onAccuracyChanged(Sensor sensor, int accuracy) {}
}
