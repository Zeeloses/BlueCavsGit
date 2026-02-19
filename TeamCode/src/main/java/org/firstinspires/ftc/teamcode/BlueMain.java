package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue TeleOp", group = "Blue")
public class BlueMain extends BlueBase {

    @Override
    protected void runAutoSequence() {
        // TeleOp uses game loop – not used
    }

    @Override
    public void runOpMode() {
        initCommonHardware();

        telemetry.addData("Status", "Online");
        telemetry.update();
        waitForStart();

        // ========== TUNING CONSTANTS – TeleOp-specific (shared from BlueBase where noted) ==========
        final double DRIVE_ACCELERATION = 0.1;    // TUNE: Ramp speed (higher = snappier)
        final double DRIVE_SLOW_SCALE = 0.5;      // TUNE: Right trigger slow-mo multiplier (0–1)
        final double FAR_LAUNCH_POWER = 0.65;     // TUNE: Launcher power for FAR shot (TeleOp only)
        final double FAR_RPM = 163;               // TUNE: Target RPM for FAR (TeleOp only)
        final double ARM_SERVO_OFF = 0.0;         // TUNE: Arm position when stopped

        // ========== STATE ==========
        Mode LAUNCH_MODE = Mode.OFF;
        double currentLeftPower = 0.0;
        double currentRightPower = 0.0;

        while (opModeIsActive()) {
            // RPM from encoders (for launcher ready LEDs, drive telemetry)
            double RPM = getLauncherRPM();
            double R_RPM = getMotorRPM(right_wheel);
            double L_RPM = getMotorRPM(left_wheel);

            // drive: left stick Y, right stick X for turn
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double targetLeftPower = drive + turn;
            double targetRightPower = drive - turn;

            double max = Math.max(
                    1.0, Math.max(Math.abs(targetLeftPower),Math.abs(targetRightPower))
            );
            targetLeftPower /= max;
            targetRightPower /= max;

            currentLeftPower = rampToward(currentLeftPower, targetLeftPower, DRIVE_ACCELERATION);
            currentRightPower = rampToward(currentRightPower, targetRightPower, DRIVE_ACCELERATION);

            double driveScale = (gamepad1.right_trigger > 0) ? DRIVE_SLOW_SCALE : 1;
            left_wheel.setPower(currentLeftPower * driveScale);
            right_wheel.setPower(currentRightPower * driveScale);

            // Arm: A = up (load), released = down (feed)
            if (gamepad1.a) setArmLoad(); else setArmFeed();

            // Launcher: Y = cycle SHORT -> FAR -> SHORT…, B = off
            if (gamepad1.yWasPressed()) {
                Mode prev = LAUNCH_MODE;
                if (prev == Mode.OFF || prev == Mode.FAR) {
                    LAUNCH_MODE = Mode.SHORT;
                }
                if (prev == Mode.SHORT) {
                    LAUNCH_MODE = Mode.FAR;
                }
            }

            if (gamepad1.bWasPressed()) {
                LAUNCH_MODE = Mode.OFF;
            }

            switch (LAUNCH_MODE) {
                case SHORT:
                    aliskLight_Green.off();
                    launcher_motor.setPower(BlueBase.LAUNCH_POWER);
                    if (BlueBase.isRPMInRange(RPM, BlueBase.LAUNCH_TARGET_RPM)) {
                        aliskLight_Red.on();
                    } else {
                        aliskLight_Red.off();
                    }
                    break;
                case FAR:
                    aliskLight_Red.off();
                    launcher_motor.setPower(FAR_LAUNCH_POWER);
                    if (BlueBase.isRPMInRange(RPM, FAR_RPM)) {
                        aliskLight_Green.on();
                    } else {
                        aliskLight_Green.off();
                    }
                    break;
                case OFF:
                    aliskLight_Green.off();
                    aliskLight_Red.off();
                    launcher_motor.setPower(0);
                    break;
            }

            // Gate: X = feed balls (only when launcher SHORT or FAR)
            if (gamepad1.x && LAUNCH_MODE != Mode.OFF) startGate(); else stopGate();
            telemetry.addData("DRIVE L", "%.2f", currentLeftPower);
            telemetry.addData("DRIVE R", "%.2f", currentRightPower);
            telemetry.addData("ARM", "%.2f", gamepad1.a ? BlueBase.ARM_LOAD : BlueBase.ARM_FEED);
            telemetry.addData("RPM","%.2f", RPM);
            telemetry.addData("R_RPM","%.2f", R_RPM);
            telemetry.addData("L_RPM","%.2f", L_RPM);
            telemetry.update();
        }

        // Cleanup (base: LEDs, launcher, gate; TeleOp adds: wheels, arm)
        cleanup();
        stopDrive();
        arm_servo.setPosition(ARM_SERVO_OFF);
    }

    /** Ramps current toward target by at most acceleration per step. Smoother starts/stops. */
    private double rampToward(double current, double target, double acceleration) {
        if (current < target) {
            return Math.min(current + acceleration, target);
        }
        if (current > target) {
            return Math.max(current - acceleration, target);
        }
        return current;
    }
}