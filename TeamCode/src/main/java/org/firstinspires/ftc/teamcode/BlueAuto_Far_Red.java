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

@Autonomous(name = "Red Far", group = "Red")
public class BlueAuto_Far_Red extends BlueBase {

    // ========== TUNING CONSTANTS – change these to affect behavior ==========
    private static final double DRIVE_POWER = 0.8;                            // TUNE: Drive power (0-1)
    private static final int DRIVE_INITIAL_MS = 1750;                         // TUNE: Distance from start to first turn (ms)
    private static final int DRIVE_APPROACH_MS = 500;                         // TUNE: Approach distance to shooting spot (ms)
    private static final TurnDirection TURN_DIRECTION = TurnDirection.RIGHT;  // TUNE: toward GOAL
    private static final int TURN_TO_GOAL_DEG = 68;                           // TUNE: Angle toward GOAL, then face for shot (deg)

    public BlueAuto_Far_Red() {
        turnSlowMultiplier = 2;  // TUNE: 2 = robot turns slow; 1 = normal
    }

    @Override
    protected void runAutoSequence() {
        driveRobot(DRIVE_POWER, DriveDirection.FORWARD, DRIVE_INITIAL_MS);
        turnRobot(TURN_DIRECTION, TURN_TO_GOAL_DEG);
        driveRobot(DRIVE_POWER, DriveDirection.FORWARD, DRIVE_APPROACH_MS);
        turnRobot(TURN_DIRECTION, TURN_TO_GOAL_DEG);
        shoot();
    }
}
