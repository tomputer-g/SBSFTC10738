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

    public void add(Vector3 a){
        this.x+=a.x;
        this.y+=a.y;
        this.z+=a.z;
    }
    public void multiply(double a){
        this.x*=a; this.y*=a; this.z*=a;
    }
    //basic operations
    public static Vector3 sum(Vector3 a, Vector3 b) {
        return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    public static Vector3 dotproduct(Vector3 a, Vector3 b){
        return new Vector3(a.x*b.x,a.y*b.y,a.z*b.z);
    }
    public static Vector3 crossproduct(Vector3 a, Vector3 b){
        return new Vector3(a.y*b.z - a.z*b.y, a.z*b.x - b.z*a.x, a.x*b.y - a.y*b.x);
    }


    //other methods
    public double dist3D(){
        return Math.sqrt(Math.sqrt(x*x+y*y)*Math.sqrt(x*x+y*y)+z*z);
    }

}
