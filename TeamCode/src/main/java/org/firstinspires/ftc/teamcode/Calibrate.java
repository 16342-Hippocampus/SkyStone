package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name="Calibrate", group="chad")
public class Calibrate extends LinearOpMode {
    //
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    //Calculate encoder conversion
    Double cpr = 537.6; //counts per rotation
    Double gearratio = 19.2;
    Double diameter = 3.93701;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference
    Double bias = 0.8;//adjust until your robot goes 20 inches
    //
    Double conversion = cpi * bias;
    //
    public void runOpMode() {
        //
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        BR.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        //
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        //
        moveToPosition(20, .2);//Don't change this line, unless you want to calibrate with different speeds
        //
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        //
        if (inches < 5) {
            int move = (int) (Math.round(inches * conversion));
            //
            FL.setTargetPosition(FL.getCurrentPosition() + move);
            FR.setTargetPosition(FR.getCurrentPosition() + move);
            BL.setTargetPosition(BL.getCurrentPosition() + move);
            BR.setTargetPosition(BR.getCurrentPosition() + move);
            //
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            FL.setPower(speed);
            FR.setPower(speed);
            BL.setPower(speed);
            BR.setPower(speed);
            //
            while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            }
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        } else {
            int move1 = (int) (Math.round((inches - 5) * conversion));
            int movefl2 = FL.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movefr2 = FR.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebl2 = BL.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebr2 = BR.getCurrentPosition() + (int) (Math.round(inches * conversion));
            //
            FL.setTargetPosition(FL.getCurrentPosition() + move1);
            FR.setTargetPosition(FR.getCurrentPosition() + move1);
            BL.setTargetPosition(BL.getCurrentPosition() + move1);
            BR.setTargetPosition(BR.getCurrentPosition() + move1);
            //
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            FL.setPower(speed);
            FR.setPower(speed);
            BL.setPower(speed);
            BR.setPower(speed);
            //
            while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            }
            //
            FL.setTargetPosition(movefl2);
            FR.setTargetPosition(movefr2);
            BL.setTargetPosition(movebl2);
            BR.setTargetPosition(movebr2);
            //
            FL.setPower(.1);
            FR.setPower(.1);
            BL.setPower(.1);
            BR.setPower(.1);
            //
            while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            }
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
        return;
    }
    /*
    ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                       |   Grave under      |
                       |   construction   |
                      ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾

                         _____________
                        /             \
                       |     CHAD      |
                       |               |
                       |  MAY HE REST  |
                       |     IN THE    |
                       |     SEWERS    |
                       |               |
    ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                                    ___
                                 |  |
                                 |  |
                                  ‾‾‾‾‾‾‾
                             \)/ .-.
                              /,(uwu)
                             ()  ( )
                      /_ ___  \\,=",
                      '-()-()   =/=\\
                       //\\||  ==== ()
                      /`  \\|  ="=  `|
                    =="    `(0V0)    '--

     */
}
