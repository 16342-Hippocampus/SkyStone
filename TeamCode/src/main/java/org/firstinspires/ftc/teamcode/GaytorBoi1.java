package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

/**
 * This is NOT an opmode.
 * This is a class that:
 *    - Contains all of your robot hardware
 *    - Does all initialization code
 *    - Has all of the methods for driving and turning
 *
 *    - This example is for the robot named Suspect, built by Astrid and Ari in 45 min
 */



public class GaytorBoi1
{
    // declare hardware imu, motors, servos, sensors
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    Servo LHook;
    Servo RHook;
    Servo StoneServo;
    //28 * 20 / (2ppi * 4.125)
    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Double gearratio = 19.2;
    Double diameter = 3.93701;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 1.01;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    // you will need a reference to your OpMode
    private LinearOpMode opModeRef;

    public GaytorBoi1(LinearOpMode opMode) {
        opModeRef = opMode;
    }

    // This is a method for all of the initialization code - all of the
    // stuff that happens after clicking the Initialize button, but before
    // clicking the start button.
    public void initialize() {
        //
        initGyro();
        //
        FL = opModeRef.hardwareMap.dcMotor.get("FL");
        FR = opModeRef.hardwareMap.dcMotor.get("FR");
        BL = opModeRef.hardwareMap.dcMotor.get("BL");
        BR = opModeRef.hardwareMap.dcMotor.get("BR");
        LHook = opModeRef.hardwareMap.servo.get("LHook");
        RHook = opModeRef.hardwareMap.servo.get("RHook");
        StoneServo = opModeRef.hardwareMap.servo.get("StoneServo");

        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        LHook.setDirection(Servo.Direction.REVERSE);
        RHook.setDirection(Servo.Direction.FORWARD);

        AutoTransitioner.transitionOnStop(opModeRef, "Drive Code V1 0 1");

    }

    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        BL.setTargetPosition(BL.getCurrentPosition() + move);
        FL.setTargetPosition(FL.getCurrentPosition() + move);
        BR.setTargetPosition(BR.getCurrentPosition() + move);
        FR.setTargetPosition(FR.getCurrentPosition() + move);
        //
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(speed);
        BR.setPower(speed);
        //
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            if (exit){
                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                return;
            }
        }
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        opModeRef.telemetry.addData("Speed Direction", speedDirection);
        opModeRef.telemetry.addData("Yaw", yaw);
        opModeRef.telemetry.update();
        //
        opModeRef.telemetry.addData("stuff", speedDirection);
        opModeRef.telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                opModeRef.telemetry.addData("Position", yaw);
                opModeRef.telemetry.addData("first before", first);
                opModeRef.telemetry.addData("first after", convertify(first));
                opModeRef.telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeRef.opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                opModeRef.telemetry.addData("Position", yaw);
                opModeRef.telemetry.addData("first before", first);
                opModeRef.telemetry.addData("first after", convertify(first));
                opModeRef.telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                opModeRef.telemetry.addData("Position", yaw);
                opModeRef.telemetry.addData("second before", second);
                opModeRef.telemetry.addData("second after", convertify(second));
                opModeRef.telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                opModeRef.telemetry.addData("Position", yaw);
                opModeRef.telemetry.addData("second before", second);
                opModeRef.telemetry.addData("second after", convertify(second));
                opModeRef.telemetry.update();
            }
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
        //</editor-fold>
        //
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        BL.setTargetPosition(BL.getCurrentPosition() - move);
        FL.setTargetPosition(FL.getCurrentPosition() + move);
        BR.setTargetPosition(BR.getCurrentPosition() + move);
        FR.setTargetPosition(FR.getCurrentPosition() - move);
        //
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(speed);
        BR.setPower(speed);
        //
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){}
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        return;
    }
    //
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */

    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = opModeRef.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        FL.setPower(input);
        BL.setPower(input);
        FR.setPower(-input);
        BR.setPower(-input);
    }

    private boolean opModeIsActive() {
        return opModeRef.opModeIsActive();
    }
    //
}
