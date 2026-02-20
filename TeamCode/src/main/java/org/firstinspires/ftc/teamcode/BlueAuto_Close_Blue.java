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

@Autonomous(name = "Blue Close", group = "Blue")
public class BlueAuto_Close_Blue extends BlueBase {

    // ========== TUNING CONSTANTS – change these to affect behavior ==========
    private static final double DRIVE_TO_SHOOT_POWER = 0.6;                  // TUNE: Power to reverse to shooting spot (0–1)
    private static final int DRIVE_TO_SHOOT_MS = 1000;                       // TUNE: Distance to shooting position (ms)

    @Override
    protected void runAutoSequence() {
        driveRobot(DRIVE_TO_SHOOT_POWER, DriveDirection.REVERSE, DRIVE_TO_SHOOT_MS);
        turnRobot(TurnDirection.LEFT, 10);
        shoot();
    }
}
