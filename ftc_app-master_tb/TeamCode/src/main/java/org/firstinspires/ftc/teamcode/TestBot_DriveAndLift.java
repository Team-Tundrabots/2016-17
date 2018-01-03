

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TestBot: DriveAndLift", group="test")

public class TestBot_DriveAndLift extends OpMode{

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

        double leftX = -gamepad1.left_stick_x;
        double leftY = -gamepad1.left_stick_y;
        double rightX = -gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y;
        double p = 0;
        double z = 0;

        // if tank drive
        robot.updateMotorsTankDrive(leftY, rightY);

        // if mechanum drive...
        //robot.updateMotorsMechanumDrive(leftX, leftY, rightX, rightY);

        telemetry.addData("leftX",  "%.2f", leftX);
        telemetry.addData("leftY",  "%.2f", leftY);
        telemetry.addData("rightX", "%.2f", rightX);
        telemetry.addData("rightY", "%.2f", rightY);
        telemetry.addData("leftClaw", "%.2f", robot.leftClaw.getPosition());

        // move lift up if a button pushed
        if(gamepad1.right_trigger > 0.5){
            robot.leftArm.setPower(.5);

        // move lift down if b button pushed
        } else if(gamepad1.left_trigger > 0.5){
            robot.leftArm.setPower(-0.5);

        // if neither button pushed, stop lift
        } else {
            robot.leftArm.setPower(0);
        }

        // move jewl bar up if a button is pushed
        if(gamepad1.a) {
            robot.leftClaw.setPosition(z);
            z = z + 0.1;
            robot.leftClaw.getPosition();
        }else if(gamepad1.b) {
            robot.leftClaw.setPosition(p);
            p = p + 0.1;
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
