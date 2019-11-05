package org.firstinspires.ftc.teamcode20.Tests;

public class Vector3 {
    //fields
    double x,y,z;

    //constructors
    public Vector3(){
        x=0; y=0; z=0;
    }
    public Vector3(double x, double y, double z){
        this.x=x; this.y=y; this.z = z;
    }

    //calculation methods
    //add to the called vector,change its value
    public void add(Vector3 a){
        this.x+=a.x;
        this.y+=a.y;
        this.z+=a.z;
    }
    //return the sum of the vectors(no change on original vectors)
    public static Vector3 sum(Vector3 a, Vector3 b) {
        return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    public void mult(double a){
        x*=a; y*=a; z*=a;
    }

    //other methods
    public double dist3D(){
        return Math.sqrt(Math.sqrt(x*x+y*y)*Math.sqrt(x*x+y*y)+z*z);
    }

}
