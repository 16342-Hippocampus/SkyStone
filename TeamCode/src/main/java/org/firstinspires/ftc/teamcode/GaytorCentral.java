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



}



