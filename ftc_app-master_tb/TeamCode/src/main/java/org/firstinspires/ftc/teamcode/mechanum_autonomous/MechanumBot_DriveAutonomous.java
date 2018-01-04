

package org.firstinspires.ftc.teamcode.mechanum_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanum.MechanumBot;


@TeleOp(name="MechanumBot Autonomous: Drive", group="mechanum")

public class MechanumBot_DriveAutonomous extends OpMode{

    /* Declare OpMode members. */
    MechanumBot robot       = new MechanumBot(telemetry);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        robot.moveForward(1.0, 0.5);
        robot.turnRight(2.0, 0.3);
        robot.moveForward(5.0, 1.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
