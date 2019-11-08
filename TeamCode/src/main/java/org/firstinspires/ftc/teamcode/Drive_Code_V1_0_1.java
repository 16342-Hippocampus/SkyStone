/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Drive Code V1 0 1", group="Linear Opmode")
public class Drive_Code_V1_0_1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor RIntake = null;
    private DcMotor LIntake = null;
    private CRServo LHook = null;
    private CRServo RHook = null;
    private TouchSensor IntakeLimit = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        RIntake =hardwareMap.get(DcMotor.class, "RIntake");
        LIntake =hardwareMap.get(DcMotor.class, "LIntake");
        LHook = hardwareMap.get(CRServo.class, "LHook");
        RHook = hardwareMap.get(CRServo.class, "RHook");
        IntakeLimit =hardwareMap.get(TouchSensor.class, "IntakeLimit");



        //GIVE ME A REASON TO LIVE
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lsx = gamepad1.left_stick_x;
            double r = Math.hypot(-lsx, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -lsx) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            double HookPower = 0;
            double IntakePower = 0;
            if(gamepad1.right_bumper){
                FL.setPower(v1/2);
                FR.setPower(v2/2);
                BL.setPower(v3/2);
                BR.setPower(v4/2);
            }else if (gamepad1.left_bumper){
                FL.setPower(v1/2);
                FR.setPower(v2/2);
                BL.setPower(v3/2);
                BR.setPower(v4/2);
            } else {
                FL.setPower(v1);
                FR.setPower(v2);
                BL.setPower(v3);
                BR.setPower(v4);
            }


            if (gamepad1.right_trigger > 0.25) {
            IntakePower = .5;
            } else if (gamepad1.left_trigger > 0.25 && !IntakeLimit.isPressed()){
                IntakePower = -1;
            } else {
                IntakePower = 0;
            }
            RIntake.setPower(IntakePower);
            LIntake.setPower(IntakePower);

            
/*            if (gamepad1.left_bumper){
                HookPower = -1;
            }else if (gamepad1.right_bumper){
                HookPower = 1;
            }else{
                HookPower = 0;
            }
            RHook.setPower(HookPower);
            LHook.setPower(HookPower);
*/

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", v1, v2, v3, v4);
            telemetry.addData ("Hooks, Hooks, Baby!", "RH (%.2f), LH (%,2f)", HookPower, HookPower);
            telemetry.addData("Intake","RI (%.2f), LI (%.2f)", IntakePower, IntakePower);
            telemetry.update();
        }
    }
}
