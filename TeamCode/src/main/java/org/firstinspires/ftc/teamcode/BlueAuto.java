/*
Copyright 2025 FIRST Tech Challenge Team 30332

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous

public class BlueAuto extends LinearOpMode {
    private DcMotor left_wheel, right_wheel;



    @Override
    public void runOpMode() {
        //HARDWARE
        left_wheel = hardwareMap.get(DcMotor.class, "left_wheel");
        right_wheel = hardwareMap.get(DcMotor.class, "right_wheel");
        //CONFIGS
        left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

        left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_wheel.setPower(0);
        right_wheel.setPower(0);
        //VARS
        boolean isForward = false;


        left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            driveRobot(30,"FORWARD",1500);
        }
    }
    int calculateTurnTime(int degrees) {
        int baseTime = (degrees*300)/90;
        if (degrees > 180) {
            baseTime = (int)(baseTime*.94);
        }
        else if (degrees > 90) {
            baseTime = (int)(baseTime*.96);
        }
        return baseTime;
    }
    void turnRobot(String direction, int degrees) {
        int power = 50;
        int turnTime = calculateTurnTime(degrees);
        if (direction.equalsIgnoreCase("left")) {
            left_wheel.setPower(-power);
            right_wheel.setPower(power);
        }
        else {
            left_wheel.setPower(power);
            right_wheel.setPower(-power);
        }
        sleep(turnTime);
        left_wheel.setPower(0);
        right_wheel.setPower(0);
    }
    void driveRobot(int power, String direction, int time) {
        telemetry.addData("sent", "sent");
        telemetry.update();
        if (direction.equalsIgnoreCase("reverse")) {
            left_wheel.setPower(-power);
            right_wheel.setPower(-power);
        }
        else {
                left_wheel.setPower(power);
                right_wheel.setPower(power);
            }
        sleep(time);
        left_wheel.setPower(0);
        right_wheel.setPower(0);

    }
}
