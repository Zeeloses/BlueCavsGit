package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close", group = "Blue")
public class BlueAuto_Close_Red extends BlueBase {

    // ========== TUNING CONSTANTS – change these to affect behavior ==========
    private static final double DRIVE_TO_SHOOT_POWER = 0.6;                  // TUNE: Power to reverse to shooting spot (0–1)
    private static final int DRIVE_TO_SHOOT_MS = 1000;                       // TUNE: Distance to shooting position (ms)

    @Override
    protected void runAutoSequence() {
        driveRobot(DRIVE_TO_SHOOT_POWER, DriveDirection.REVERSE, DRIVE_TO_SHOOT_MS);
        shoot();
    }
}
