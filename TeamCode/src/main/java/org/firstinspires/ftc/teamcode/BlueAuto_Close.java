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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class BlueAuto_Close extends LinearOpMode {
    private DcMotor left_wheel, right_wheel, launcher_motor, gate_motor;
    private Servo arm_servo;
    private IMU imu;

    // P Control Constants - TUNE THESE VALUES
    private final double TURN_KP = 0.02;      // Turn proportional gain
    private final double DRIVE_KP = 0.015;    // Drive heading correction gain
    private final double TURN_TOLERANCE = 2.0;  // Degrees of acceptable error
    private final double MIN_TURN_POWER = 0.15; // Minimum power to overcome friction
    private final double MAX_TURN_POWER = 0.6;  // Maximum turn power

    @Override
    public void runOpMode() {
        //HARDWARE
        left_wheel = hardwareMap.get(DcMotor.class, "left_wheel");
        right_wheel = hardwareMap.get(DcMotor.class, "right_wheel");
        launcher_motor = hardwareMap.get(DcMotor.class, "launcher_motor");
        gate_motor = hardwareMap.get(DcMotor.class, "gate_motor");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        imu = hardwareMap.get(IMU.class, "imu");

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

        left_wheel.setPower(0);
        right_wheel.setPower(0);

        left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // hub pos on robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("status", "init");
        telemetry.addData("IMU", "calibrated");
        telemetry.update();
        waitForStart();

        // calibrate
        imu.resetYaw();
        arm_servo.setPosition(-1);



        if (opModeIsActive()) {
            driveRobot(0.6, "reverse", 1000);
            sleep(1000);
            shoot();
            turnRobot("left",63);  // Turn left 63 degrees
            driveRobot(0.5, "forward", 1250);
        }
    }

    void shoot() {
        final double LAUNCH_MAIN_POWER = .78;
        final double GATE_POWER = .75;

        launcher_motor.setPower(LAUNCH_MAIN_POWER);
        sleep(4000);

        arm_servo.setPosition(-.5);
        //1st
        gate_motor.setPower(GATE_POWER);
        sleep(2500);
        sleep(750);
        //2nd
        arm_servo.setPosition(1); // arm
        sleep(2750);
        arm_servo.setPosition(-.5);
        sleep(750);
        gate_motor.setPower(0);
        //3rd
        arm_servo.setPosition(1); // arm
        sleep(2750);
        gate_motor.setPower(GATE_POWER);
        arm_servo.setPosition(-.5);
        sleep(500);
        arm_servo.setPosition(1);
        sleep(2750);
        arm_servo.setPosition(-.5);
        sleep(1000);
        gate_motor.setPower(0);
    }

    int calculateTurnTime(int degrees)
    {
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

    void driveRobot(double power, String direction, int time) {
        telemetry.addData("sent", "sent");
        telemetry.update();
        if (direction.equalsIgnoreCase("reverse")) {
            left_wheel.setPower(-power);
            right_wheel.setPower(-power);
        } else {
            left_wheel.setPower(power);
            right_wheel.setPower(power);
        }
        sleep(time);
        left_wheel.setPower(0);
        right_wheel.setPower(0);
    }
}