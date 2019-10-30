package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous(name="Calibrate3", group="chad")
public class Calibrate3 extends LinearOpMode {
    //
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    //Calculate encoder conversion
    Double cpr = 537.6; //counts per rotation
    Double gearratio = 19.2;
    Double diameter = 4.0;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference
    Double bias = 0.8;//adjust until your robot goes 20 inches
    //
    Double conversion = cpi * bias;
    //
    public void runOpMode() {
        //
        BR = hardwareMap.get(DcMotor.class, "BR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
            BR.setTargetPosition(BR.getCurrentPosition() + move);

            //
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //
            BR.setPower(speed);

            //
            while (BR.isBusy()) {
            }
            BR.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        } else {
            int move1 = (int) (Math.round((inches - 5) * conversion));
            int movefl2 = BR.getCurrentPosition() + (int) (Math.round(inches * conversion));
            //
            BR.setTargetPosition(BR.getCurrentPosition() + move1);

            //
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //
            BR.setPower(speed);
            //
            while (BR.isBusy()) {
            }
            //
            BR.setTargetPosition(movefl2);
            //
            BR.setPower(.1);
            //
            while (BR.isBusy()) {
            }
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
