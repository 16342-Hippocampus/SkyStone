package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class GaytorBoi{

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor FR = null;
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;
    public DcMotor RIntake = null;
    public DcMotor LIntake = null;
    public CRServo LHook = null;
    public CRServo RHook = null;
    public TouchSensor IntakeLimit = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public GaytorBoi(){

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
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

    }
}

