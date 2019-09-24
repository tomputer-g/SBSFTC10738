package org.firstinspires.ftc.teamcode;
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
	double[][] points = {{0,0},{1,2},{2,3},{3,0}};
	double[][] result = SplineGenerate.splinefunction(points);
	double[] theta={0,0};
	double[] v=new double[2];
	double[] p={0,0};
	ElapsedTime t;
	int i=0;
	@Override
	public void init() {
		for(double[] j: result)
			telemetry.addLine(j[0]+" "+j[1]+" "+j[2]+" "+j[3]+" "+j[4]);
		telemetry.update();
		v[0]=1;

	}

	@Override
	public void loop() {
		if(near(p[0],result[i][0],0.01)&&near(p[1],result[i][1],0.01))
			i++;
		if(i==points.length-1)
			requestOpModeStop();
		v[1]=result[i][1]*(t.milliseconds()/1000-points[i-1][0])+2*result[i][2]*(t.milliseconds()/1000-points[i-1][0])+3*result[i][3]*(t.milliseconds()/1000-points[i-1][0])*(t.milliseconds()/1000-points[i-1][0]);
		theta[1]=Math.atan(v[1]);

		//turn(theta[1]-theta[0]);
		setAllDrivePower(v[0],v[1]);
		theta[0]=theta[1];
		p[0]+=v[0];
		p[1]+=v[1];
	}
}
