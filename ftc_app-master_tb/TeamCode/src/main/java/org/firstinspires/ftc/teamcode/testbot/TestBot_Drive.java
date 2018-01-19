

package org.firstinspires.ftc.teamcode.testbot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TestBot: Drive", group="test")
@Disabled

public class TestBot_Drive extends OpMode{

    /* Declare OpMode members. */
    TestBot robot       = new TestBot(telemetry);

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

        double leftX = gamepad1.left_stick_x;
        double leftY = gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        // if tank drive
        robot.updateMotorsTankDrive(leftY, rightY);

        // if mechanum drive...
        //robot.updateMotorsMechanumDrive(leftX, leftY, rightX, rightY);

        telemetry.addData("leftX",  "%.2f", leftX);
        telemetry.addData("leftY",  "%.2f", leftY);
        telemetry.addData("rightX", "%.2f", rightX);
        telemetry.addData("rightY", "%.2f", rightY);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
