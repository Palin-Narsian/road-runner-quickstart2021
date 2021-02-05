/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.MecanumTeleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30,0,2,15);
    // Main Motors
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    // Wobble Goal Arm
    DcMotor WobbleArm;
    Servo wobbleServo;
    Servo test;
    DcMotorEx shooter;
    DcMotor intake;
    protected VoltageSensor batteryVoltageSensor;

    //variables
    double WobblePositionOpen;
    double wobbleClose = 0;
    double wobbleExtend = -2150;
    double wobbleExtendHalfway = -800;
    double wobbleExtendOverWall = -1600;
    double wobbleServoCount = 2;
    double chassiSpeed;
    private ElapsedTime runtime = new ElapsedTime();
    double  shooterVar = 0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lf = hwMap.get(DcMotor.class, "lf");
        rf = hwMap.get(DcMotor.class, "rf");
        lb = hwMap.get(DcMotor.class, "lb");
        rb = hwMap.get(DcMotor.class, "rb");
        intake = hwMap.get(DcMotor.class, "intake");
        WobbleArm = hwMap.get(DcMotor.class, "WobbleArm");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        wobbleServo = hwMap.get(Servo.class, "wobbleServo");
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        test = hwMap.servo.get("test");
        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d, SHOOTER_PID.f*12/hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));
        // Set all motors to zero power
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        lf.setMode(DcMotor. RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        test.setPosition(0);
        WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleServo.setPosition(1);

        chassiSpeed = 1;

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Define and initialize ALL installed servos.

    }
    public void MecanumTeleopDrive() {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        lf.setPower(v1);
        rf.setPower(v2);
        lb.setPower(v3);
        rb.setPower(v4);
    }
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx shooter) {
            shooter.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

}

