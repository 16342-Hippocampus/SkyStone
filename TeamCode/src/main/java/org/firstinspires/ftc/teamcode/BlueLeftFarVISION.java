package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//@Disabled
@Autonomous(name="BlueLeftFarVISION", group="chad")
public class BlueLeftFarVISION extends LinearOpMode {
    //
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    Servo LHook;
    Servo RHook;
    Servo StoneServo;

    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation of the final output
    Double gearratio = 19.2;
    Double diameter = 3.93701;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 1.01;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement. DO NOT ADJUST THIS OR ALL OF THE STRAFING IN THE AUTO WILL NEED TO BE ADJUSTED!!!
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK; //I'm not sure if this does anything useful, but I'm scared to delete it.
    private static final String VUFORIA_KEY =
            "AbvJs+3/////AAABmecKc3at703Lj8ticYWbWq08wQWXZRotCtbpEjsV/mUeKYwdDdKIj9bVhTZTpHPKevMhV+WVTiIO+VW8uz0aLR5eahMx+STrPONwTjwxe38V5cmPwh8RHkB4Eu8J9Jd1kqST3Sy5/J0oNT23qImslwYm4nyxiqqbjyUI4JcwBU14slrzEnZSLsFcU48fQia9vXnUhbmmVyRwiIdF3BvXOhkJH5pc8nwnuPz9VXV6EFuk4RsliIeUiWjWsxc8rhfOImLoqcQ6l3Xh4WcNYr3TQYiBuCxVuwhKS5ulCcrhY/IiOxsfNe/PjdCK1SbmKao8U0UDine9Cxi9/qe+w5nIkqHuxx+oBpE0BSsfKKF2qhA5";
    //That key is the developer key I got off of the vuforia website to get it to activate.
    // Class Members
    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null; //This indicates which camera on the robot controller we want to use.


    public void runOpMode(){
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //this retrieves the webcam we are going to use.
        //
        initGyro();
        //
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        LHook = hardwareMap.servo.get("LHook");
        RHook = hardwareMap.servo.get("RHook");
        StoneServo = hardwareMap.servo.get("StoneServo");

        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        LHook.setDirection(Servo.Direction.REVERSE);
        RHook.setDirection(Servo.Direction.FORWARD);
        //
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CAMERA_CHOICE;
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        //AutoTransitioner.transitionOnStop(this, "Drive Code V1 0 1");

        waitForStartify();

      /*  LHook.setPosition(1);
        RHook.setPosition(1);
        //
        moveToPosition(-2, 0.2);
        //
        strafeToPosition(13.0, 0.2);
        //
        moveToPosition(-25, 0.5);
        moveToPosition(-6, 0.2);
        //
        LHook.setPosition(.91);
        RHook.setPosition(.91);
        sleep(500);
        //
        moveToPosition(50, 1);
        //
        LHook.setPosition(1);
        RHook.setPosition(1);
        sleep(500);
        //
        strafeToPosition(20.0, 0.5);
        strafeToPosition(-50.0, 0.5);
        moveToPosition(-25, 0.2);
        //
        turnWithGyro(89, -0.2);
        //
        moveToPosition(55, 0.5);
        // All this code is commented out because I needed to specifically work on the vision part of the code without the robot moving.
        //Since the vision code works, this needs to be uncommented and then edited
        //so that the robot stops consistently in one place and we can tune the offset values
        //once the offset values are tuned, we can start creating paths that the robot will run based on the 3 possible combinations of stones.
*/
        targetsSkyStone.activate();
        //I'm not sure this is the correct while loop. I might instead want this loop to run for a couple seconds and then
        //move on to code further down so that it's no longer running commands based on what its seeing so that it doesnt get confused.
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                OpenGLMatrix location = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getVuforiaCameraFromTarget();

                if (location != null) {
                    // Get the positional part of the coordinates
                    VectorF translation = location.getTranslation();
                    //clip the actual X to see if it is closer to the left or right
                    float closestX = Range.clip(translation.get(0), -20f, 20f);
                    /*"center" because we (my team) only looks at the right two in the farthest set of three in the quarry,
                    so the leftmost image would be the center of the three stones concerned */
                    //If using the phone, it should be oriented horizontally with the buttons up
                    //With the camera
                    if (closestX == -20) {
                        telemetry.addData("Skystone Target:", "Center");
                        //this tells us that the stones are arranged a particular way.
                        //now we can run a bunch of movement commands because we know what arrangement pattern the skystones are.
                    }
                    //Right most stone of the two
                    if (closestX == 20) {
                        telemetry.addData("Skystone Target:", "Right");

                    }
                    //Also express the relative pose (for info purposes)
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0), translation.get(1), translation.get(2));
                }
                telemetry.update();

            } else {
                telemetry.addData("Visible Target", "none");
                telemetry.update();

            }


        }
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
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
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
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
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
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
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
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
    public void waitForStartify(){
        waitForStart();
    }
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
    //
}

