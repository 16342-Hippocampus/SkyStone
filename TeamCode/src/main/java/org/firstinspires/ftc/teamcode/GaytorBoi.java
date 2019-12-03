package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class GaytorBoi{

    private LinearOpMode OpModeReference;

    double ticks = 537.6;
    double wheelDiameter = 3.93701;
    double countsPerInch = ticks/(wheelDiameter*Math.PI);
    public DcMotor[] AllMotors = new DcMotor[4];

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor FR = null;
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;
    public DcMotor RIntake = null;
    public DcMotor LIntake = null;
    public CRServo LHook = null;
    public CRServo RHook = null;
    public Servo StoneServo = null;
    public TouchSensor IntakeLimit = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public GaytorBoi(LinearOpMode opMode){
        OpModeReference = opMode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FR  = hwMap.get(DcMotor.class, "FR");
        FL = hwMap.get(DcMotor.class, "FL");
        BR    = hwMap.get(DcMotor.class, "BR");
        BL    = hwMap.get(DcMotor.class, "BL");
        RIntake =hwMap.get(DcMotor.class, "RIntake");
        LIntake =hwMap.get(DcMotor.class, "LIntake");
        LHook = hwMap.get(CRServo.class, "LHook");
        RHook = hwMap.get(CRServo.class, "RHook");
        StoneServo = hwMap.get(Servo.class, "StoneServo");
        IntakeLimit =hwMap.get(TouchSensor.class, "IntakeLimit");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        RIntake.setDirection(DcMotor.Direction.FORWARD);
        LIntake.setDirection(DcMotor.Direction.REVERSE);
        LHook.setDirection(CRServo.Direction.FORWARD);
        RHook.setDirection(CRServo.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        RIntake.setPower(0);
        LIntake.setPower(0);
        LHook.setPower(0);
        RHook.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.

        //Array time!!!
        AllMotors[0] = FL;
        AllMotors[1] = FR;
        AllMotors[2] = BL;
        AllMotors[3] = BR;
    }

    public void DouglessDrive(double inches, double speed) {
        int target = (int)(inches * countsPerInch);
        for (DcMotor m : AllMotors){
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(speed/2);
        }

        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            OpModeReference.telemetry.addData("FL", FL.getCurrentPosition());
            OpModeReference.telemetry.addData("FR", FR.getCurrentPosition());
            OpModeReference.telemetry.addData("BL", BL.getCurrentPosition());
            OpModeReference.telemetry.addData("BR", BR.getCurrentPosition());
            OpModeReference.telemetry.update();
        }

        for (DcMotor m : AllMotors){
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }public void DougDrive(double inches, double speed) {
        speed = Range.clip(speed, 0.25, 0.5);
        int target = (int)(inches * countsPerInch);
        for (DcMotor m : AllMotors){
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(speed);
        }

        int FLprev = 99;
        int FRprev = 99;
        int BLprev = 99;
        int BRprev = 99;

        int FLdif = 99;
        int FRdif = 99;
        int BLdif = 99;
        int BRdif = 99;


        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            if (FLprev == 99)
                FLprev = FL.getCurrentPosition();
            else {
                FLprev = FL.getCurrentPosition();
                FLdif = Math.abs(FLprev - FL.getCurrentPosition());
            }
            if (FRprev == 99)
                FRprev = FR.getCurrentPosition();
            else {
                FRprev = FR.getCurrentPosition();
                FRdif = Math.abs(FRprev - FR.getCurrentPosition());
            }
            if (BLprev == 99)
                BLprev = BL.getCurrentPosition();
            else {
                BLprev = BL.getCurrentPosition();
                BLdif = Math.abs(BLprev - BL.getCurrentPosition());
            }
            if (BRprev == 99)
                BRprev = BR.getCurrentPosition();
            else {
                BRprev = BR.getCurrentPosition();
                BRdif = Math.abs(BRprev - BR.getCurrentPosition());
            }
            int average;

            if (FLdif != 99) {
                average = Math.round((FLdif + FRdif + BLdif + BRdif)/4);
            }

            //UNFINISHED!!!
        }

        for (DcMotor m : AllMotors){
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

}

