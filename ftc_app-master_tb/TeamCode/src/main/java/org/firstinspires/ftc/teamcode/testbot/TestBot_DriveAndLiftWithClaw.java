

package org.firstinspires.ftc.teamcode.testbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TestBot: DriveAndLiftWithClaw", group="test")

public class TestBot_DriveAndLiftWithClaw extends OpMode{

    /* Declare OpMode members. */
    TestBot robot       = new TestBot(telemetry);


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

        // if tank drive
        robot.updateMotorsTankDrive(leftY, rightY);

        // if mechanum drive...
        //robot.updateMotorsMechanumDrive(leftX, leftY, rightX, rightY);

        telemetry.addData("leftX",  "%.2f", leftX);
        telemetry.addData("leftY",  "%.2f", leftY);
        telemetry.addData("rightX", "%.2f", rightX);
        telemetry.addData("rightY", "%.2f", rightY);

        // move arm up if a button pushed
        if(gamepad1.right_trigger > 0){
            robot.leftArm.setPower(-0.5);

        // move arm down if b button pushed
        } else if(gamepad1.left_trigger > 0){
            robot.leftArm.setPower(.5);

        // if neither button pushed, stop arm
        } else {
            robot.leftArm.setPower(0);
        }

        // Use dpad to open and close the claw
        if (gamepad1.dpad_up)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.dpad_down)
            clawOffset -= CLAW_SPEED;

        robot.moveClaws(clawOffset);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
