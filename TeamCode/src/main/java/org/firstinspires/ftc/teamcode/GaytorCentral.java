package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.ElapsedTime;
public class GaytorCentral {

    private LinearOpMode OpModeReference;

    public GaytorCentral(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public void ColorInit() {
        blinkinLedDriver = OpModeReference.hardwareMap.get(RevBlinkinLedDriver.class, "PrettyBoi");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);

    }

    public void ColorPlayground() {
        OpModeReference.telemetry.addData("Color Time?", "Goin'?");
        if (OpModeReference.gamepad2.a) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
            OpModeReference.telemetry.addData("Color Time?", "Lava Wave");
        } else if (OpModeReference.gamepad2.b) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            OpModeReference.telemetry.addData("Color Time?", "Forest Wave");
        } else if (OpModeReference.gamepad2.x) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            OpModeReference.telemetry.addData("Color Time?", "Ocean Wave");
        } else if (OpModeReference.gamepad2.y) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
            OpModeReference.telemetry.addData("Color Time?", "Party Wave");
        } else if (OpModeReference.gamepad2.dpad_left) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            OpModeReference.telemetry.addData("Color Time?", "WHITE STROBE");
        } else if (OpModeReference.gamepad2.dpad_right) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            OpModeReference.telemetry.addData("Color Time?", "BLUE STROBE");
        } else if (OpModeReference.gamepad2.dpad_down) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            OpModeReference.telemetry.addData("Color Time?", "RED STROBE");
        } else if (OpModeReference.gamepad2.dpad_up) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            OpModeReference.telemetry.addData("Color Time?", "GOLD STROBE");
        } else if (OpModeReference.gamepad2.right_bumper) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
            OpModeReference.telemetry.addData("Color Time?", "Rainbow Wave");
        } else if (OpModeReference.gamepad2.left_bumper) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
            OpModeReference.telemetry.addData("Color Time?", "Glitter Rainbow");
        }

    }


    public void GaytorInit(){
        ElapsedTime runtime = new ElapsedTime();
        DcMotor FR = null;
        DcMotor FL = null;
        DcMotor BR = null;
        DcMotor BL = null;
        DcMotor FRIntake = null;
        DcMotor FLIntake = null;
        CRServo Roller = null;
        Servo LHook = null;
        Servo RHook = null;
        // Servo StoneServo = null;
        // DcMotor BRIntake = null;
        // DcMotor BLIntake = null;
        // TouchSensor IntakeLimit = null;
        CRServo LOuttake;
        CRServo ROuttake;
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "FR");
        FL = OpModeReference.hardwareMap.get(DcMotor.class, "FL");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "BR");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "BL");
        FRIntake =OpModeReference.hardwareMap.get(DcMotor.class, "FRIntake");
        FLIntake =OpModeReference.hardwareMap.get(DcMotor.class, "FLIntake");
        //BRIntake =OpModeReference.hardwareMap.get(DcMotor.class, "BRIntake");
        //BLIntake =OpModeReference.hardwareMap.get(DcMotor.class, "BLIntake");
        Roller = OpModeReference.hardwareMap.get(CRServo.class, "Roller");
        LHook = OpModeReference.hardwareMap.get(Servo.class, "LHook");
        RHook = OpModeReference.hardwareMap.get(Servo.class, "RHook");
        //StoneServo = OpModeReference.hardwareMap.get(Servo.class, "StoneServo");
        //IntakeLimit =OpModeReference.hardwareMap.get(TouchSensor.class, "IntakeLimit");
        LOuttake =OpModeReference.hardwareMap.get(CRServo.class, "LOuttake");
        ROuttake =OpModeReference.hardwareMap.get(CRServo.class, "ROuttake");

        //GIVE ME A REASON TO LIVE
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FRIntake.setDirection(DcMotor.Direction.REVERSE);
        FLIntake.setDirection(DcMotor.Direction.FORWARD);
        //BRIntake.setDirection(DcMotor.Direction.REVERSE);
        //BLIntake.setDirection(DcMotor.Direction.FORWARD);
        ROuttake.setDirection(DcMotor.Direction.REVERSE);
        LOuttake.setDirection(DcMotor.Direction.FORWARD);

        LHook.setDirection(Servo.Direction.REVERSE);
        RHook.setDirection(Servo.Direction.FORWARD);
        //StoneServo.setDirection(Servo.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //BRIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //BLIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)



    }

}



