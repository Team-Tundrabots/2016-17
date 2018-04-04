

package org.firstinspires.ftc.teamcode.simple;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SimpleBot: Drive", group="simple")
//@Disabled

public class SimpleBot_Drive extends OpMode{

    /* Declare OpMode members. */
    SimpleBot robot       = new SimpleBot(telemetry);

    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

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

        robot.updateMotorsTankDrive(leftY, rightY);

        double position = robot.leftDrive.getCurrentPosition();
        // Send telemetry message to signify robot running;
        telemetry.addData("leftX", "%.2f", leftX);
        telemetry.addData("leftY", "%.2f", leftY);
        telemetry.addData("rightX", "%.2f", rightX);
        telemetry.addData("rightY", "%.2f", rightY);
        if(gamepad1.a)
        {
            telemetry.addData("encoder", "%.2f", position);
            robot.leftDrive.setTargetPosition(-1000);
            robot.leftDrive.setPower(1.0);

        }

        // Use dpad to open and close the claw
        if (gamepad1.dpad_up)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.dpad_down)
            clawOffset -= CLAW_SPEED;

        //robot.moveClaws(clawOffset);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
