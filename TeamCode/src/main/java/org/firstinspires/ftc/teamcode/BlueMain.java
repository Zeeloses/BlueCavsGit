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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;


enum Mode {
    SHORT,
    FAR,
    OFF
}
/**
 * +
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class BlueMain extends LinearOpMode {
    //DRIVE MOTOR
    private DcMotor gate_motor;
    private DcMotorEx launcher_motor, left_wheel, right_wheel;
    private Servo arm_servo;
    private LED aliskLight_Red, aliskLight_Green;

    @Override
    public void runOpMode() {
        //HARDWARE LEGEND
        left_wheel = hardwareMap.get(DcMotorEx.class, "left_wheel");
        right_wheel = hardwareMap.get(DcMotorEx.class, "right_wheel");
        launcher_motor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        gate_motor = hardwareMap.get(DcMotor.class, "gate_motor");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        aliskLight_Red = hardwareMap.get(LED.class, "aliskLight_Red");
        aliskLight_Green = hardwareMap.get(LED.class, "aliskLight_Green");

        //MOTOR DIRECTIONS ARE OPPOSITE TO SIDE
        left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        gate_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //BREAK BEHAVIOR
        left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gate_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TURN ON ENCODER
        launcher_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Online");
        telemetry.update();
        waitForStart();

        //TUNING CONSTANTS
        final double DRIVE_ACCELERATION = 0.1; // Amount to change speed per loop iteration
        final double LAUNCH_MAIN_POWER = 0.5;
        final double GATE_POWER = 0.75;
        final double ARM_SERVO = 1.0;
        final double SHORT_RPM = 130;
        final double FAR_RPM = 163;

        //VARS
        Mode LAUNCH_MODE = Mode.OFF;
        double currentLeftPower = 0.0;  // Add these to track current speed
        double currentRightPower = 0.0;

        while (opModeIsActive()) {
            //RPM BASE CALCULATION - RUNS IN LOOP AND IS CONSTANTLY UPDATED
            double ticksPerSec = launcher_motor.getVelocity();
            double ticksPerRev = launcher_motor.getMotorType().getTicksPerRev();
            double RPM = calcRPM(ticksPerSec,ticksPerRev);

            double R_ticksPerSec = right_wheel.getVelocity();
            double R_ticksPerRev = right_wheel.getMotorType().getTicksPerRev();
            double R_RPM = calcRPM(R_ticksPerSec,R_ticksPerRev);

            double L_ticksPerSec = left_wheel.getVelocity();
            double L_ticksPerRev = left_wheel.getMotorType().getTicksPerRev();
            double L_RPM = calcRPM(L_ticksPerSec,L_ticksPerRev);

            // MAIN PROCESS VARS
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double targetLeftPower = drive + turn;
            double targetRightPower = drive - turn;

            double max = Math.max(
                    1.0, Math.max(Math.abs(targetLeftPower),Math.abs(targetRightPower))
            );
            targetLeftPower /= max;
            targetRightPower /= max;

            if (currentLeftPower < targetLeftPower) {
                currentLeftPower = Math.min(currentLeftPower + DRIVE_ACCELERATION, targetLeftPower);
            } else if (currentLeftPower > targetLeftPower) {
                currentLeftPower = Math.max(currentLeftPower - DRIVE_ACCELERATION, targetLeftPower);
            }

            if (currentRightPower < targetRightPower) {
                currentRightPower = Math.min(currentRightPower + DRIVE_ACCELERATION, targetRightPower);
            } else if (currentRightPower > targetRightPower) {
                currentRightPower = Math.max(currentRightPower - DRIVE_ACCELERATION, targetRightPower);
            }

            double driveScale = (gamepad1.right_trigger > 0) ? .5 : 1;
            left_wheel.setPower(currentLeftPower * driveScale);
            right_wheel.setPower(currentRightPower * driveScale);

            //arm_servo configs
            double armPower = -0.5;
            boolean aButton = gamepad1.a;
            if (aButton) {
                armPower = 1.0;
            }
            arm_servo.setPosition(armPower);

            //LAUNCHER MOTOR CONTROLS
            if (gamepad1.yWasPressed()) {
                Mode localMode = LAUNCH_MODE; //temp set to avoid double access of var

                if (localMode == Mode.OFF || localMode == Mode.FAR) {
                    LAUNCH_MODE = Mode.SHORT;
                }

                if (localMode == Mode.SHORT){
                    LAUNCH_MODE = Mode.FAR;
                }
            }

            if(gamepad1.bWasPressed()){
                LAUNCH_MODE = Mode.OFF;
            }

            if (LAUNCH_MODE == Mode.SHORT) {
                aliskLight_Green.off();
                launcher_motor.setPower(.55);
                if (RPM >= SHORT_RPM && RPM <= (SHORT_RPM + 20)) {
                    aliskLight_Red.on();
                } else {
                    aliskLight_Red.off();
                }
            }

            if (LAUNCH_MODE == Mode.FAR) {
                aliskLight_Red.off();
                launcher_motor.setPower(.65);
                if (RPM >= FAR_RPM && RPM <= (FAR_RPM + 20)) {
                    aliskLight_Green.on();
                } else {
                    aliskLight_Green.off();
                }
            }

            if (LAUNCH_MODE == Mode.OFF){
                aliskLight_Green.off();
                aliskLight_Red.off();
                launcher_motor.setPower(0.0);
            }

            //GATE MOTOR CONTROLS
            gate_motor.setPower(gamepad1.x ? GATE_POWER : 0.0);
            //telemetry
            telemetry.addData("DRIVE L", "%.2f", currentLeftPower);
            telemetry.addData("DRIVE R", "%.2f", currentRightPower);
            telemetry.addData("ARM_SERVO", "%.2f", armPower);
            telemetry.addData("RPM","%.2f", RPM);
            telemetry.addData("R_RPM","%.2f", R_RPM);
            telemetry.addData("L_RPM","%.2f", L_RPM);
            telemetry.update();
        }

        //cleanup
        left_wheel.setPower(0.0);
        right_wheel.setPower(0.0);
        arm_servo.setPosition(0.0);
        launcher_motor.setPower(0.0);
    }

    public double calcRPM(double TPS, double TPV) {
        return (TPS / TPV) * 60;
    }
}