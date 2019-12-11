package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@Autonomous(name="Blue Left Close 2", group="chad")
public class BlueLeftClose2 extends LinearOpMode {

    private GaytorBoi1 gaytorBoi1 = new GaytorBoi1(this);

    public void runOpMode(){
        gaytorBoi1.initialize();
        //
        waitForStart();

        //Raise the hooks
        gaytorBoi1.LHook.setPosition(1);
        gaytorBoi1.RHook.setPosition(1);
        //Move a tad away from the wall
        gaytorBoi1.moveToPosition(-2, 0.2);
        //Mash into the wall in zealous glory
        gaytorBoi1.strafeToPosition(13.0, 0.2);
        //Move the bulk of the distance to the foundation
        gaytorBoi1.moveToPosition(-25, 0.5);
        //Gently mash the robot into the foundation
        gaytorBoi1.moveToPosition(-6, 0.2);
        //Lower the hooks
        //gaytorBoi1.LHook.setPosition(.91);
        //gaytorBoi1.RHook.setPosition(.91);
        sleep(500);
        //Pull the foundation back to the wall
        gaytorBoi1.moveToPosition(50, 0.5);
        //
        gaytorBoi1.LHook.setPosition(1);
        gaytorBoi1.RHook.setPosition(1);
        sleep(500);
        //
        gaytorBoi1.strafeToPosition(20.0, 0.5);
        gaytorBoi1.strafeToPosition(-65.0, 0.5);
        gaytorBoi1.strafeToPosition(-11.0, 0.2);
        //
    }
    //


}

