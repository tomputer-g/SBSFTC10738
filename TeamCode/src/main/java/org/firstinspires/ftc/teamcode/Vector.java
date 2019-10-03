package org.firstinspires.ftc.teamcode;

public class Vector {
    //fields
    double x,y,z;

    //constructors
    public Vector(){
        x=0; y=0; z=0;
    }
    public Vector(double x, double y){
        this.x=x; this.y=y; z=0;
    }
    public Vector(double x, double y, double z){
        this.x=x; this.y=y; this.z = z;
    }

    //calculation methods
    //add to the called vector,change its value
    public void add(Vector a){
        this.x+=a.x;
        this.y+=a.y;
        this.z+=a.z;
    }
    //return the sum of the vectors(no change on original vectors)
    public static Vector sum(Vector a, Vector b) {
        return new Vector(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    public void mult(double a){
        x*=a; y*=a; z*=a;
    }

    //other methods
    public double dist(){
        return Math.sqrt(Math.sqrt(x*x+y*y)*Math.sqrt(x*x+y*y)+z*z);
    }
    public double dist3D(){
        return Math.sqrt(Math.sqrt(x*x+y*y)*Math.sqrt(x*x+y*y)+z*z);
    }
    public double theta(){
        return x>=0?Math.atan(y/x):Math.PI+Math.atan(y/x);
    }
    public Vector toPolar(){
        return new Vector(theta(),dist());
    }
}
