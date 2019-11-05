package org.firstinspires.ftc.teamcode20.Tests;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode19.BaseOpMode;

/**
 * Test Program for the Spline Generator
 * Version sep.10th
 * Barry Lyu
*/
@Autonomous(group = "test")
public class SplineTest extends BaseOpMode {
	double[][] points = {{0,0},{5,10},{10,15},{15,0}};
	double[][] result = SplineGenerate.splinefunction(points);
	double[] theta={0,0};
	double[] v=new double[2];
	double[] p={0,0};
	double speed,t;
	int i=0;
	@Override
	public void init() {
		for(double[] j: result)
			telemetry.addLine(j[0]+" "+j[1]+" "+j[2]+" "+j[3]+" "+j[4]);
		telemetry.update();
		v[0]=1;
		speed=0;
		t=0;
	}
	@Override
	public void loop() {
		if(near(p[0],points[i][0],0.01)&&near(p[1],points[i][1],0.01))
			i++;
		if(i==points.length-1)
			requestOpModeStop();
		v[0]=1;
		v[1]=3*result[i][0]*Math.pow((t-result[i][3]),2)+2*result[i][2]*(t-result[i][4])+result[i][1];
		//theta[1]=Math.atan(v[1]);
		//turn(theta[1]-theta[0]);
		//moveInches(Math.sqrt(Math.pow(v[0]*0.01,2)+Math.pow(v[1]*0.01,2)),0,speed);
		//theta[0]=theta[1];
		moveInches(v[0]*0.01,v[1]*0.01,speed);
		p[0]+=v[0];
		p[1]+=v[1];
		t+=0.01;
	}
}
