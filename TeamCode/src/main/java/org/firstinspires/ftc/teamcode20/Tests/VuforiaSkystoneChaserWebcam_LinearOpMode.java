package org.firstinspires.ftc.teamcode20.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static java.lang.Math.sqrt;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@TeleOp
public class VuforiaSkystoneChaserWebcam_LinearOpMode extends LinearOpMode {

    protected DcMotor LF, LB, RF, RB;
    //Vuforia
    protected static final float mmPerInch = 25.4f;
    protected OpenGLMatrix lastLocation = null;
    protected VuforiaTrackables targetsSkyStone;
    protected List<VuforiaTrackable> allTrackables;
    private final float CAMERA_FORWARD_DISPLACEMENT  = 0f * mmPerInch;//2.5 in from end + 1 in correction
    private final float CAMERA_VERTICAL_DISPLACEMENT = 0f * mmPerInch;// eg: Camera is 8 Inches above ground
    private final float CAMERA_LEFT_DISPLACEMENT     = 0f * mmPerInch; // eg: Camera is ON the robot's center line

    //IMU
    protected static BNO055IMU imu;
    protected static double imuHeading;
    protected static double imuOffset;

    private double[] displacements = {2, 7};//+ = forward; + = right
    private double headingDisplacement = -90;

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforiaWebcam();
        initDrivetrain();
        initIMU();
        targetsSkyStone.activate();
        waitForStart();
        while(opModeIsActive()){
            boolean targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    if(trackable.getName().equals("Stone Target")) {
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addLine("Turn "+(int)Math.abs(rotation.thirdAngle+headingDisplacement)+(rotation.thirdAngle+headingDisplacement>0?"deg. CW":"deg. CCW"));
                        turn(-(rotation.thirdAngle+headingDisplacement),0.3,2);//+=ccw
                        VectorF translation = lastLocation.getTranslation();
                        telemetry.addLine("Move "+Math.abs(translation.get(1)/mmPerInch + displacements[1])+(translation.get(1)+displacements[1]>0?"in. Right":"in. Left"));
                        telemetry.addLine("Forward "+(-translation.get(0)/mmPerInch + displacements[0])+"in.");
                        moveInches(translation.get(1)/mmPerInch+displacements[1], translation.get(0)/mmPerInch+displacements[0], 0.4);
                    }
                    break;
                }
            }
            telemetry.update();
        }
        targetsSkyStone.deactivate();
    }

    protected void moveInches(double xInch, double yInch, double speed){
        /*
        double xmult = 14./2, ymult = 14./2, p_mult = 80;

        int encoder_x = (int)(xInch * xmult), encoder_y = (int)(yInch * ymult);
        int encoder_1 = Math.abs(encoder_x + encoder_y); // LB, RF
        int encoder_2 = Math.abs(encoder_x - encoder_y); // LF, RB
        double conversion_fct = speed/((encoder_1 + encoder_2)/2);
        double speed_1 = conversion_fct * encoder_1, speed_2 = conversion_fct * encoder_2;
        setAllDrivePower(speed_2,speed_1,speed_1,speed_2);
        setAllDrivePower(-speed,-speed,speed,speed);
        telemetry.addData("speed",speed_1+" "+speed_2);
        telemetry.addData("position",encoder_x+" "+encoder_y);
        telemetry.update();
        LF.setTargetPosition(encoder_x - encoder_y);
        LB.setTargetPosition(-encoder_x - encoder_y);
        RF.setTargetPosition(encoder_x + encoder_y);
        RB.setTargetPosition(-encoder_x + encoder_y);
        setMode_RESET_AND_RUN_TO_POSITION();
        while((LF.isBusy()||LB.isBusy()||RF.isBusy()||RB.isBusy()) && t.milliseconds() < p_time){
            telemetry.addData("Power", LF.getPower());
            telemetry.update();
        };
        setAllDrivePower(0);
        ();

        */

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode_RUN_WITHOUT_ENCODER();
        ElapsedTime t = new ElapsedTime();
        int p_time = (int) (sqrt(xInch*xInch + yInch*yInch)*100);
        double xmult = 14./1.2, ymult = 14./1.2;
        int encoder_x = (int)(xInch * xmult), encoder_y = (int)(yInch * ymult);
        //double theta=Math.atan(xInch/yInch);
        //double vy=Math.cos(theta)*speed,vx=Math.sin(theta)*speed;
        double coe=1;
        while(encoder_x - encoder_y<-LF.getCurrentPosition()||-encoder_x - encoder_y<-LB.getCurrentPosition()||encoder_x + encoder_y>-RF.getCurrentPosition()||-encoder_x + encoder_y>-RB.getCurrentPosition()){
            telemetry.addData("LF",-LF.getCurrentPosition());
            telemetry.addData("target",encoder_x-encoder_y);

            telemetry.addData("LB",-LB.getCurrentPosition());
            telemetry.addData("target",-encoder_x-encoder_y);

            telemetry.addData("RF",-RF.getCurrentPosition());
            telemetry.addData("target",encoder_x+encoder_y);

            telemetry.addData("RB",-RB.getCurrentPosition());
            telemetry.addData("target",-encoder_x+encoder_y);

            telemetry.update();
            //if (p_time < t.milliseconds()) break;
            setAllDrivePower(-coe*speed,-coe*speed,coe*speed,coe*speed);
            //coe+=
        }
        setAllDrivePower(1,1,-1,-1);
        wait(100);
        setAllDrivePower(0);
    }

    protected void wait(int time){
        sleep(time);
    }

    protected void setAllDrivePower(double pLF, double pLB, double pRF, double pRB){
        LF.setPower(pLF);
        LB.setPower(pLB);
        RF.setPower(pRF);
        RB.setPower(pRB);
    }

    protected void initDrivetrain(){
        LF = hardwareMap.get(DcMotor.class,"LF");
        LB = hardwareMap.get(DcMotor.class,"LB");
        RF = hardwareMap.get(DcMotor.class,"RF");
        RB = hardwareMap.get(DcMotor.class,"RB");
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void initVuforiaWebcam(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZlAJSf/////AAABmf5BgAyil0t8tK506wQNabJX0SH5ekkamom8UybSLKgtsYTY/0/AB5n0Db9/JRrUDLEhDRXJgx5osNHZt6kVKSIF5cdge/dE9OgOunoX6LWBqk8cHGwBlKCXl1eGuvBPwQa3OaJDC7neKLmlZf2/NJiJKMvi9VBqKEDsS74Dp0tFbJka5cJa8YpKyrJh8593SN8p2qcYxXRORCWzmdMdD2xHUJXw28foxuNOotp2onbDmpnfH7x4oegFalegxvQbJ3J0cFqOuP8pboEjoN0Zl64xFVu6ZCc2uvsnXECEgWtycA+bWmQZNG6BD4SLYN/LWVYBp6U5MrIHsNeOOQfwTAZNVDcLELke77iK1XuWnCzG";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, 2.00f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, 23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 59, 180)));
        blueRearBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, 23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -59, 180)));
        redFrontBridge.setLocation(OpenGLMatrix.translation(-5.18f * mmPerInch, -23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -59, 0)));
        redRearBridge.setLocation(OpenGLMatrix.translation(5.18f * mmPerInch, -23 * mmPerInch, 6.42f * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 59, 0)));
        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix.translation(36 * mmPerInch, -72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        red2.setLocation(OpenGLMatrix.translation(-36 * mmPerInch, -72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        front1.setLocation(OpenGLMatrix.translation(-72 * mmPerInch, -36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        front2.setLocation(OpenGLMatrix.translation(-72 * mmPerInch, 36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        blue1.setLocation(OpenGLMatrix.translation(-36 * mmPerInch, 72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        blue2.setLocation(OpenGLMatrix.translation(36 * mmPerInch, 72 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        rear1.setLocation(OpenGLMatrix.translation(72 * mmPerInch, 36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        rear2.setLocation(OpenGLMatrix.translation(72 * mmPerInch, -36 * mmPerInch, 6 * mmPerInch).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, 0,0,0));


        for (VuforiaTrackable trackable : allTrackables)
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    protected void initIMU(){
        BNO055IMU.Parameters BNOParameters = new BNO055IMU.Parameters();
        BNOParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        BNOParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        BNOParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        BNOParameters.loggingEnabled = true;
        BNOParameters.loggingTag = "imu";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(BNOParameters);
    }

    protected void getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        imuHeading = Double.parseDouble(String.format(Locale.getDefault(), "%.2f", AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)))) - imuOffset;
    }

    protected void setNewGyro0(){
        imuOffset = 0;
        getHeading();
        imuOffset = imuHeading;
    }

    protected void setMode_RUN_WITHOUT_ENCODER(){//DO NOT USE ANY OTHER RUNMODES. GoBilda 5202 series motors have weird encoders
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void turn(double angle, double speed, double threshold) {
        setMode_RUN_WITHOUT_ENCODER();
        setNewGyro0();
        double p_TURN = 0.25;
        while(!onHeading(speed, angle, p_TURN, threshold));
    }

    private boolean onHeading(double turnSpeed, double angle, double PCoeff, double threshold) {
        double   error = getError(angle), steer, speed;
        boolean  onTarget = false;

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            speed = 0.0;
            onTarget = true;
            telemetry.addData("ON TARGET!", error);
        }
        else {
            steer = Range.clip(error * PCoeff, -1, 1);
            speed  = turnSpeed * steer;
        }
        setAllDrivePower(speed);
        telemetry.update();
        return onTarget;
    }

    private double getError(double targetAngle) {
        getHeading();
        double robotError = targetAngle - imuHeading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    protected void setAllDrivePower(double power){
        LF.setPower(power);
        LB.setPower(power);
        RF.setPower(power);
        RB.setPower(power);
    }
}