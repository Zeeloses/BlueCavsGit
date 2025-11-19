package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Blue Playground")
public class Blue_Team_Playground extends LinearOpMode{
    private DcMotor left_drive_wheel, right_drive_wheel;

    double maxSpeed;
    double minSpeed;
    double targetSpeed;
    double currentSpeed;
    double accelRate;

    @Override
    public void runOpMode() {
        // Set Motors
        left_drive_wheel = hardwareMap.get(DcMotor.class,"left_wheel");
        // right_drive_wheel = hardwareMap.get(DcMotor.class,"right_wheel");

        left_drive_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // right_drive_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set vars
        minSpeed = .5;
        maxSpeed = .8;
        targetSpeed = 0;
        currentSpeed = 0;
        accelRate = 5;


        // for (DcMotor m: new DcMotor[]{left_drive_wheel, right_drive_wheel}){
        //     m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "in Opmode");
            telemetry.update();
            targetSpeed = checkDeadband(-gamepad1.left_stick_y, minSpeed);

            // double turn = checkDeadband(-gamepad1.right_stick_x, minSpeed);

            // double leftPower = drive + turn;
            // double rightPower = drive - turn;

            if (targetSpeed != 0)
            {
                currentSpeed += targetSpeed/accelRate;

                currentSpeed = (currentSpeed > targetSpeed) ? currentSpeed = targetSpeed : currentSpeed;

                if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
                else if(currentSpeed < -maxSpeed) currentSpeed = -maxSpeed;

                left_drive_wheel.setPower(currentSpeed);
            }
            else
            {
                currentSpeed -= maxSpeed/accelRate;
                if (currentSpeed < minSpeed)
                {
                    currentSpeed = 0;
                }
                left_drive_wheel.setPower(currentSpeed);
            }
            telemetry.addData("Speed", currentSpeed);
            telemetry.update();
            sleep(100);

        }
    }

    private double checkDeadband(double stickPos, double threshold)
    {
        if (Math.abs(stickPos) < threshold)
            return 0;
        return stickPos;
    }

    // todo: write your code here
}
