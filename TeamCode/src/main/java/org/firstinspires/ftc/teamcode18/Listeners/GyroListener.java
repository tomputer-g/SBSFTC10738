package org.firstinspires.ftc.teamcode18.Listeners;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

/**
 * Created by Ziming Gao on 12/5/2017.
 */

public class GyroListener implements SensorEventListener {
    public double y = 0.0;
    @Override public void onSensorChanged(SensorEvent event) {
        y = event.values[1];//CCW = positive, CW = negative
    }

    @Override public void onAccuracyChanged(Sensor sensor, int accuracy) {}
}
