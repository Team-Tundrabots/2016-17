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

package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="CompetitionBot: AutoDriveBlueGlyph", group="competition")
//@Disabled
public class CompetitionBot_AutoDriveBlueGlyph extends LinearOpMode {

    /* Declare OpMode members. */
    CompetitionBot  robot   = new CompetitionBot(telemetry, this);

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.initCamera();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.leftArm.setPower(-0.5); // lift raises
        sleep(2000);
        robot.moveClaws(0.5); // open claws
        robot.leftArm.setPower(0);
        sleep(2000);
        //robot.leftArm.setPower(0.5);
        //sleep(2000);
        robot.leftArm.setPower(0);
        robot.moveClaws(-0.3);


        double tailPosition = robot.tail.getPosition();
//        robot.tail.setPosition(tailPosition -.03);
        robot.tail.setPosition(0.2);
        sleep(1000);

        String color =  robot.getColorFromCamera();
        if(color == "RED") {
            robot.turnRight(1.1, 1.0);
            robot.tail.setPosition(1);
            robot.moveBackward(1.5, 1);
            telemetry.addData("color", "RED");

        }
        else
        {
            robot.turnLeft(0.9, 1);
            robot.tail.setPosition(1);
            robot.moveForward(1, 1);
            telemetry.addData("color", "BLUE");
        }


        telemetry.addData("tailPosition",  "%.2f", tailPosition);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
