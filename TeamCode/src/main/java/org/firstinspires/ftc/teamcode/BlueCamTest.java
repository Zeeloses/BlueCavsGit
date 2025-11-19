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

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
//ID 1: BLUE
//ID 2: RED
//ID 3,4,5: OBELISK

public class BlueCamTest extends LinearOpMode {
    private DcMotor left_wheel, right_wheel, launcher_motor;
    private Servo arm_servo;
    // CONFIGS
    private HuskyLens camLens;
    // CONST
    final int TEAM_TAG = 1;
    // VARS
    private boolean aligned = false;

    @Override

    public void runOpMode() {
        camLens = hardwareMap.get(HuskyLens.class, "cam_lens");

        if (!initHuskyLens()) {
            telemetry.addData("Not Inited","Not ready to go");
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // boolean aPressed = gamepad1.left_bumper;
            // if (aPressed){
            HuskyLens.Block[] blocks = camLens.blocks();
            aligned = alignRobot(blocks);
            // telemetry.addData("blocks", Arrays.toString(blocks));
            // }


            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

    private boolean initHuskyLens() {
        try{
            if (!camLens.knock()){
                return false;
            }
            camLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            return true;
        } catch(Exception e) {
            return false;
        }
    }
    private boolean alignRobot(HuskyLens.Block[] blocks) {
        while (!aligned && opModeIsActive()) {
            for (HuskyLens.Block tag : blocks){
                if (tag.id == TEAM_TAG) { //ALIGN THE ROBOT TO BLUE ID
                    if (tag.x > 160){
                        telemetry.addData("Turn", "Right");
                    }
                    else {
                        telemetry.addData("Turn", "Left");
                    }
                }
                else break;
                return false;
            }
            telemetry.update();
        }
        return true;
    }
}
