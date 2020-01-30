package org.firstinspires.ftc.teamcode20.Tests;

/**
 * Spline Generator program
 * version Sep.10th
 * inference from: https://en.wikipedia.org/wiki/Spline_(mathematics)
 * Barry Lyu
 * 
 * **/

public class SplineGenerate {
	//the function for the spline
	public static double[][] splinefunction(double[][] points) {
		int n = points.length-1;
		double[] a = new double[n+1];
		for(int i = 0; i <= n; i++)
			a[i]=points[i][1];
		double[] h = new double[n];	
		for(int i = 0; i<n;i++)
			h[i] = points[i+1][0]-points[i][0];
		double[] alpha = new double[n];
		for(int i = 1; i<n;i++)
			alpha[i] = 3*(a[i+1]-a[i])/h[i]-3*(a[i]-a[i-1])/h[i-1];
		double[] l = new double[n+1];
		l[0] = 1;
		double[] z = new double[n+1];
		z[0] = 0;
		double[] mu = new double[n+1];
		mu[0] = 0;
		for(int i = 1; i<n;i++) {
			l[i] = 2*(points[i+1][0]-points[i-1][0])-h[i-1]*mu[i-1];
			mu[i] = h[i]/l[i];
			z[i] = (alpha[i]-h[i-1]*z[i-1])/l[i];
		}
		l[n]=1;
		z[n]=0;
		double[] c = new double[n+1];
		double[] b = new double[n];
		double[] d = new double[n];
		c[n]=0;
		for(int i = n-1; i>=0 ; i--){
			c[i]=z[i]-mu[i]*c[i+1];
			b[i]=(a[i+1]-a[i])/h[i]-h[i]*(c[i+1]+2*c[i])/3;
			d[i]= (c[i+1]-c[i])/(3*h[i]);
		}
		//returning the parameters in pairs in formula (a+b(x-e)+c(x-e)^2+d(x-e)^3) for each two adjacent points
        double[][] spline = new double[n][5];
		for (int i = 0; i < n;i++){
			spline[i][0] = a[i];
			spline[i][1] = b[i];
			spline[i][2] = c[i];
			spline[i][3] = d[i];
			spline[i][4] = points[i][0]; //this is e in the formula
		}
		return spline;
	}
}
