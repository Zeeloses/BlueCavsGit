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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Shared hardware init, launcher/shooter logic,
 * drive/turn helpers, and LED behavior.
 */
public abstract class BlueBase extends LinearOpMode {

    // Shared hardware – subclasses use these
    protected DcMotorEx left_wheel, right_wheel, launcher_motor;
    protected DcMotor gate_motor;
    protected Servo arm_servo;
    protected LED aliskLight_Red, aliskLight_Green;

    // Shared tuning constants – same across autos and TeleOp (public so BlueMain can import)
    public static final double LAUNCH_POWER = 0.55;               // Launcher motor power (0–1). Higher = faster spin.
    public static final double GATE_POWER = 0.75;                 // Gate/feeder motor power (0–1). Higher = faster ball feed.
    public static final double ARM_FEED = -0.5;                   // Arm servo position when feeding ball (-1 to 1).
    public static final double ARM_LOAD = 1.0;                    // Arm servo position when loading next ball (-1 to 1).
    protected static final double ARM_INIT = -1.0;                // Arm position at auto start (ready for 1st ball).
    protected static final int NUM_BALLS = 4;                     // Number of pre-loaded balls to shoot in autonomous.
    public static final double LAUNCH_TARGET_RPM = 130;           // Target RPM before feeding. Red LED on when in range.
    public static final double LAUNCH_RPM_TOLERANCE = 25;         // ±RPM window for "ready" (e.g. 130±20 = 130–150).

    /** Returns true if rpm is within LAUNCH_RPM_TOLERANCE above target. Used by shoot, waitForLauncherReady, TeleOp. */
    protected static boolean isRPMInRange(double rpm, double targetRPM) {
        return rpm >= targetRPM && rpm <= targetRPM + LAUNCH_RPM_TOLERANCE;
    }

    protected static final int ARM_CYCLE_MS = 2750;               // Time (ms) for arm to move up or down.
    protected static final int TURN_MS_PER_90_DEG = 300;          // Base time (ms) per 90° turn. Formula: deg × this / 90.
    protected static final double TURN_FACTOR_OVER_90 = 0.96;     // Multiplier for turns >90° (compensation).
    protected static final double TURN_FACTOR_OVER_180 = 0.94;    // Multiplier for turns >180° (compensation).

    /** Subclasses set this – used by calculateTurnTime (1 = normal, 2 = slower). */
    protected int turnSlowMultiplier = 1;
    /** Subclasses set this – used by turnRobot and driveRobot. */
    protected double turnPower = 0.5;

    /**
     * Template: init -> onInit -> waitForStart -> runAutoSequence -> cleanup.
     * ADJUST: Override onInit or runAutoSequence; do not override this unless you need a different flow.
     * TROUBLESHOOT: OpMode doesn't run -> check Driver Station config; stops early -> check opModeIsActive in loops.
     */
    @Override
    public void runOpMode() {
        initCommonHardware();
        onInit();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            runAutoSequence();
        }
        cleanup();
    }

    /**
     * Override to add setup before waitForStart (e.g. IMU init).
     * Default: sets arm to ARM_INIT (ready for 1st ball). Call super.onInit() if overriding.
     * ADJUST: ARM_INIT. TROUBLESHOOT: Arm wrong at start -> adjust ARM_INIT; IMU drift -> check hub orientation.
     */
    protected void onInit() {
        arm_servo.setPosition(ARM_INIT);
    }

    /**
     * Override with your auto's drive/turn/shoot sequence.
     * ADJUST: Call driveRobot(), turnRobot(), shoot() with your path constants.
     * TROUBLESHOOT: Wrong path -> change drive/turn times and degrees in your auto class constants.
     */
    protected abstract void runAutoSequence();

    /**
     * Loads motors, servos, LEDs from hardwareMap and sets directions/modes.
     * ADJUST: Change hardware names if config differs; change setDirection if robot wired opposite.
     * TROUBLESHOOT: Motor spins wrong way -> flip that motor's setDirection. "Unknown hardware" -> names must match
     * Robot Config. Motors don't move -> check breakers, battery, Control Hub connection.
     */
    protected void initCommonHardware() {
        left_wheel = hardwareMap.get(DcMotorEx.class, "left_wheel");
        right_wheel = hardwareMap.get(DcMotorEx.class, "right_wheel");
        launcher_motor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        gate_motor = hardwareMap.get(DcMotor.class, "gate_motor");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        aliskLight_Red = hardwareMap.get(LED.class, "aliskLight_Red");
        aliskLight_Green = hardwareMap.get(LED.class, "aliskLight_Green");

        left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        gate_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gate_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_wheel.setPower(0);
        right_wheel.setPower(0);
        launcher_motor.setPower(0);
        stopGate();

        left_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gate_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Stops launcher, gate, turns off LEDs.
     * ADJUST: Override to add wheel stop, arm position, or other cleanup.
     * TROUBLESHOOT: Motors keep running after stop -> override cleanup() and add wheel/arm stop.
     */
    protected void cleanup() {
        aliskLight_Red.off();
        aliskLight_Green.off();
        launcher_motor.setPower(0);
        stopGate();
    }

    // ---------- Shared methods ----------

    /**
     * Shoots NUM_BALLS: waits for RPM, feeds only when in range, stops when RPM dips (ball through).
     * ADJUST: LAUNCH_POWER, GATE_POWER, ARM_FEED, ARM_LOAD, NUM_BALLS, LAUNCH_TARGET_RPM, LAUNCH_RPM_TOLERANCE, ARM_CYCLE_MS.
     * TROUBLESHOOT: Ball doesn't feed -> increase GATE_POWER or check ARM_FEED position. Feeds too fast -> decrease
     * GATE_POWER. Red LED never on -> increase LAUNCH_POWER or lower LAUNCH_TARGET_RPM. Shoots too early -> decrease
     * LAUNCH_RPM_TOLERANCE. Ball detected too soon (double-feed) -> increase LAUNCH_RPM_TOLERANCE or add debounce.
     */
    protected void shoot() {
        launcher_motor.setPower(LAUNCH_POWER);
        waitForLauncherReady();

        for (int i = 0; i < NUM_BALLS; i++) {
            if (i > 0) {
                waitForLauncherReady();
                stopGate();
                cycleArm();
            }
            feedOneBall();
        }
    }

    /**
     * Runs gate only when launcher RPM in range; stops when RPM drops (ball through). Arm set to ARM_FEED first.
     * ADJUST: GATE_POWER, ARM_FEED, LAUNCH_TARGET_RPM, LAUNCH_RPM_TOLERANCE.
     * TROUBLESHOOT: Gate never runs -> launcher not at RPM. Gate runs too long -> check RPM dip detection.
     */
    protected void feedOneBall() {
        setArmFeed();
        boolean ballFed = false;
        while (opModeIsActive() && !ballFed) {
            updateLauncherLED();
            double rpm = getLauncherRPM();
            if (isRPMInRange(rpm, LAUNCH_TARGET_RPM)) {
                startGate();
            } else {
                ballFed = true;
                stopGate();
            }
            sleep(50);
        }
        stopGate();
    }

    /** Turns gate motor on at GATE_POWER. */
    protected void startGate() {
        gate_motor.setPower(GATE_POWER);
    }

    /** Turns gate motor off. */
    protected void stopGate() {
        gate_motor.setPower(0);
    }

    /** Sets arm to feed position (ball ready to launch). */
    protected void setArmFeed() {
        arm_servo.setPosition(ARM_FEED);
    }

    /** Sets arm to load position (grabbing next ball). */
    protected void setArmLoad() {
        arm_servo.setPosition(ARM_LOAD);
    }

    /**
     * Blocks until launcher RPM is in target range.
     * ADJUST: LAUNCH_TARGET_RPM, LAUNCH_RPM_TOLERANCE.
     * TROUBLESHOOT: Never returns -> increase LAUNCH_POWER or lower LAUNCH_TARGET_RPM. LED flickers -> widen LAUNCH_RPM_TOLERANCE.
     */
    protected void waitForLauncherReady() {
        while (opModeIsActive()) {
            updateLauncherLED();
            if (isRPMInRange(getLauncherRPM(), LAUNCH_TARGET_RPM)) {
                return;
            }
            sleep(50);
        }
    }

    /**
     * Returns launcher speed as RPM. Uses encoder – no adjustment needed unless motor type changes.
     * TROUBLESHOOT: RPM reads 0 or wrong -> check launcher uses RUN_USING_ENCODER; verify encoder cable.
     */
    protected double getLauncherRPM() {
        return getMotorRPM(launcher_motor);
    }

    /**
     * Returns RPM for any DcMotorEx. Use for launcher, wheels, or other encoded motors.
     * ADJUST: None – uses encoder. TROUBLESHOOT: RPM wrong -> check motor uses RUN_USING_ENCODER.
     */
    protected double getMotorRPM(DcMotorEx motor) {
        double ticksPerSec = motor.getVelocity();
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        return (ticksPerSec / ticksPerRev) * 60;
    }

    /**
     * Sets red LED on when RPM in range, off otherwise. Matches TeleOp behavior.
     * ADJUST: LAUNCH_TARGET_RPM, LAUNCH_RPM_TOLERANCE.
     * TROUBLESHOOT: LED never on -> launcher not reaching RPM (see waitForLauncherReady). LED flickers -> widen
     * LAUNCH_RPM_TOLERANCE. LED stays on when stopped -> check RPM calc or motor still spinning.
     */
    protected void updateLauncherLED() {
        aliskLight_Green.off();
        double rpm = getLauncherRPM();
        if (isRPMInRange(rpm, LAUNCH_TARGET_RPM)) {
            aliskLight_Red.on();
        } else {
            aliskLight_Red.off();
        }
    }

    /**
     * Sleeps for ms while updating launcher LED every 50ms.
     * ADJUST: Change 50ms sleep interval if LED updates feel too fast/slow.
     * TROUBLESHOOT: Arm/gate timing feels off -> ARM_CYCLE_MS is used in armUp, not here.
     */
    protected void sleepWithLedUpdate(int ms) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (opModeIsActive() && t.milliseconds() < ms) {
            updateLauncherLED();
            sleep(50);
        }
    }

    /**
     * Moves arm up to load, then down to feed. Used for ball transitions.
     * ADJUST: ARM_LOAD, ARM_FEED, ARM_CYCLE_MS (via armUp).
     * TROUBLESHOOT: Next ball doesn't load -> increase ARM_CYCLE_MS or adjust ARM_LOAD. Ball drops early ->
     * decrease ARM_CYCLE_MS or check ARM_FEED. Arm hits mechanism -> reduce ARM_LOAD travel.
     */
    protected void cycleArm() {
        armUp();
        setArmFeed();
    }

    /**
     * Moves arm to load position and waits ARM_CYCLE_MS.
     * ADJUST: ARM_LOAD, ARM_CYCLE_MS.
     * TROUBLESHOOT: Arm doesn't reach ball -> increase ARM_CYCLE_MS or move ARM_LOAD further. Arm overshoots ->
     * decrease ARM_LOAD. Next ball not ready -> increase ARM_CYCLE_MS.
     */
    protected void armUp() {
        setArmLoad();
        sleepWithLedUpdate(ARM_CYCLE_MS);
    }

    /**
     * Computes turn duration (ms) from degrees. Used by turnRobot.
     * ADJUST: TURN_MS_PER_90_DEG, TURN_FACTOR_OVER_90, TURN_FACTOR_OVER_180, turnSlowMultiplier.
     * TROUBLESHOOT: Robot overshoots turn -> decrease TURN_MS_PER_90_DEG or decrease TURN_FACTOR (e.g. 0.94->0.92).
     * Undershoots -> increase TURN_MS_PER_90_DEG. Turns too slow overall -> set turnSlowMultiplier=1 in subclass.
     */
    protected int calculateTurnTime(int degrees) {
        int baseTime = (degrees * TURN_MS_PER_90_DEG) / 90;
        if (degrees > 180) {
            baseTime = (int) (baseTime * TURN_FACTOR_OVER_180);
        } else if (degrees > 90) {
            baseTime = (int) (baseTime * TURN_FACTOR_OVER_90);
        }
        return baseTime * turnSlowMultiplier;
    }

    /**
     * Turns robot in direction by degrees, then stops.
     * ADJUST: turnPower (field), or pass different direction/degrees from runAutoSequence.
     * TROUBLESHOOT: Overshoots -> decrease turnPower. Undershoots -> increase turnPower or degrees. Inconsistent ->
     * check wheel slip, battery level, turnSlowMultiplier in subclass.
     */
    protected void turnRobot(TurnDirection direction, int degrees) {
        int turnTime = calculateTurnTime(degrees);
        if (direction == TurnDirection.LEFT) {
            left_wheel.setPower(-turnPower);
            right_wheel.setPower(turnPower);
        } else {
            left_wheel.setPower(turnPower);
            right_wheel.setPower(-turnPower);
        }
        sleep(turnTime);
        stopDrive();
    }

    /**
     * Drives forward or reverse at power for time (ms), then stops.
     * ADJUST: Pass power (0–1), direction, and time from your auto constants.
     * TROUBLESHOOT: Drives too far -> decrease time (ms). Not far enough -> increase time or power. Curves to side ->
     * check wheel directions; consider matching power to compensate for imbalance.
     */
    protected void driveRobot(double power, DriveDirection direction, int time) {
        if (direction == DriveDirection.REVERSE) {
            left_wheel.setPower(-power);
            right_wheel.setPower(-power);
        } else {
            left_wheel.setPower(power);
            right_wheel.setPower(power);
        }
        sleep(time);
        stopDrive();
    }

    /**
     * Stops both drive wheels. No tuning needed.
     * TROUBLESHOOT: Robot coasts after stop -> check setZeroPowerBehavior(BRAKE) in initCommonHardware.
     */
    protected void stopDrive() {
        left_wheel.setPower(0);
        right_wheel.setPower(0);
    }
}
