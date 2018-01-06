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

package org.firstinspires.ftc.teamcode.simple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

public class SimpleBot {
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    public DcMotor leftDrive2 = null;
    public DcMotor rightDrive2 = null;

    public DcMotor leftArm = null;

    public Servo leftClaw = null;
    public Servo rightClaw = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    Telemetry telemetry = null;
    LinearOpMode opMode = null;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public SimpleBot() {

    }

    public SimpleBot(Telemetry atelemetry) {
        telemetry = atelemetry;
    }

    public SimpleBot(Telemetry atelemetry, LinearOpMode aOpMode) {
        this(atelemetry);
        opMode = aOpMode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = initMotor(hwMap, "left_drive", DcMotor.Direction.REVERSE);
        rightDrive = initMotor(hwMap, "right_drive");

        leftDrive2 = initMotor(hwMap, "left_drive2", DcMotor.Direction.REVERSE);
        rightDrive2 = initMotor(hwMap, "right_drive2");

        leftArm = initMotor(hwMap, "left_arm");

        // Define and initialize ALL installed servos.
        leftClaw = initServo(hwMap, "left_claw");
        rightClaw = initServo(hwMap, "right_claw");
    }


    public void setHwMap(HardwareMap ahwMap) {
        hwMap = ahwMap;
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public DcMotor initMotor(HardwareMap ahwMap, String motorName) {
        return(initMotor(ahwMap, motorName, DcMotor.Direction.FORWARD));
    }

    public DcMotor initMotor(HardwareMap ahwMap, String motorName, DcMotorSimple.Direction direction) {

        try {
            DcMotor motor = ahwMap.get(DcMotor.class, motorName);

            motor.setDirection(direction);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            return (motor);

        } catch (IllegalArgumentException err) {
            if (telemetry != null) {
                telemetry.addData("Error", err.getMessage());
            }
            return null;
        }
    }

    public Servo initServo(HardwareMap ahwMap, String servoName) {
        try {
            Servo servo = ahwMap.get(Servo.class, servoName);
            servo.setPosition(0.5);

            return (servo);

        } catch (IllegalArgumentException err) {
            if (telemetry != null) {
                telemetry.addData("Error", err.getMessage());
            }
            return null;
        }
    }

    public void updateMotorsTankDrive(double leftY, double rightY) {

        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -leftY;
        right = -rightY;

        setLeftMotorsPower(left);
        setRightMotorsPower(right);

    }

    public void moveClaws(double clawOffset) {
        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        if (leftClaw != null)
            leftClaw.setPosition(0.5 + clawOffset);
        if (rightClaw != null)
            rightClaw.setPosition(0.5 - clawOffset);

    }

    public void move(double seconds, double leftPower, double rightPower) {

        ElapsedTime runtime = new ElapsedTime();

        setLeftMotorsPower(leftPower);
        setRightMotorsPower(rightPower);

        runtime.reset();
        while(opMode.opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        setLeftMotorsPower(0.0);
        setRightMotorsPower(0.0);

    }

    public void moveForward(double seconds, double power)
    {
        move(seconds,power, power);
    }

    public void moveBackward(double seconds, double power)
    {
        // move forwards with negative power
        move(seconds, -power, -power);
    }

    public void turnLeft(double seconds, double power) {
        move(seconds, -power, power);
    }

    public void turnRight(double seconds, double power) {
        move(seconds, power, -power);
    }

    public void wait(double seconds) {
        long milliseconds = (long)seconds * 1000;
        opMode.sleep(milliseconds);
    }

    public void setLeftMotorsPower(double power){
        if (leftDrive != null) {
            leftDrive.setPower(power);
        }
        if (leftDrive2 != null) {
            leftDrive2.setPower(power);
        }
    }

    public void setRightMotorsPower(double power){
        if (rightDrive != null) {
            rightDrive.setPower(power);
        }
        if (rightDrive2 != null) {
            rightDrive2.setPower(power);
        }
    }
}

