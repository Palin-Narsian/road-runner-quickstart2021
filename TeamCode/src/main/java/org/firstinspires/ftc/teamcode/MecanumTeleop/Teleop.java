package org.firstinspires.ftc.teamcode.MecanumTeleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumTeleop.HardwarePushbot;

import static java.lang.Thread.sleep;


/**
 * Created by robot on 9/17/2018.
 */
@TeleOp(name = " 9356 Mecanum Ultimate Goal")

public class Teleop extends OpMode
{

    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    double WobblePositionOpen; //0=fully in 1= fully extended 2= halfway up 3= extended in consideration for wall

    @Override
    public void init()  {
        robot.init(hardwareMap);
        //shooter = hardwareMap.dcMotor.get("shooter");
    }

    @Override
    public void loop() {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.lf.setPower(v1*robot.chassiSpeed);
        robot.rf.setPower(v2*robot.chassiSpeed);
        robot.lb.setPower(v3*robot.chassiSpeed);
        robot.rb.setPower(v4*robot.chassiSpeed);
        //wobble stuff


        if (gamepad1.b) {
            robot.shooterVar = 0;
        }
        if (gamepad1.right_bumper) {
            robot.shooterVar = -0.225;
        }

        if (gamepad1.left_bumper) {
            robot.shooterVar = -0.233;
        }

        if (gamepad1.a) {
            robot.test.setPosition(0.4);
            try {
                sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.test.setPosition(0);
        }

        if (gamepad1.left_trigger>0 && gamepad1.right_trigger==0) {
            robot.intake.setPower(1);
        }

        if (gamepad1.left_trigger==0 && gamepad1.right_trigger==0) {
            robot.intake.setPower(0);
        }

        if (gamepad1.right_trigger>0&&gamepad1.left_trigger==0) {
            robot.intake.setPower(-1);
        }
        if (gamepad1.dpad_left) {
            robot.test.setPosition(0);
        }
        telemetry.addData("power", robot.shooter.getVelocity());
        robot.shooter.setVelocity(robot.shooterVar);


        if (gamepad1.x) {
            robot.chassiSpeed = 1;
        }

        if (gamepad1.y) {
            robot.chassiSpeed = 0.2;
        }

        if (gamepad2.a) {
            robot.wobbleServo.setPosition(1);
        }

        if (gamepad2.b) {
            robot.wobbleServo.setPosition(0);
        }

        robot.WobbleArm.setPower(-gamepad2.left_stick_y*0.1);

        //endgame automation. Not sure if it will work.
//        if (gamepad2.a) {
////            try {
////                EndGameWobbleAutomation();
////            } catch (InterruptedException e) {
////                e.printStackTrace();
////            }
////        }




       /* double shooterVar = 0;
        if (gamepad1.a) {
            shooterVar = 1;
        }
        if (gamepad1.b) {
            shooterVar = 0;
        }
        if (gamepad1.x) {
            shooterVar = -1;
        }
        shooter.setPower(shooterVar);*/
    }
    public void WobbleMove(double speed, double target, double timeoutS) {
        int Wobbletarget = (int) target;

        robot.WobbleArm.setTargetPosition(Wobbletarget);
        robot.WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.WobbleArm.setPower(Math.abs(speed));
        while (
                (runtime.seconds() < timeoutS) &&
                        (robot.WobbleArm.isBusy())) {
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.lf.setPower(v1*robot.chassiSpeed);
            robot.rf.setPower(v2*robot.chassiSpeed);
            robot.lb.setPower(v3*robot.chassiSpeed);
            robot.rb.setPower(v4*robot.chassiSpeed);
        }
        robot.WobbleArm.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.WobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void EndGameWobbleAutomation() throws InterruptedException {
        WobblePositionOpen = 3;
        WobbleMove(0.5, robot.wobbleExtendOverWall, 100);//move arm over wall
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.wobbleServo.setPosition(1); //open servo
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.lf.setPower(-0.4); //move backward
        robot.rf.setPower(-0.4);
        robot.lb.setPower(-0.4);
        robot.rb.setPower(-0.4);
        sleep(500);

        robot.lf.setPower(0); //stop so robot doesn't go flying
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
        sleep(10);
    }
    public void checkFlywheelRPM() throws InterruptedException {
        double initialPosition = robot.shooter.getCurrentPosition();
        sleep(250);
        double afterPosition = robot.shooter.getCurrentPosition();

        double totalRevs = (afterPosition-initialPosition)/7;
        double RPM = totalRevs * 4 *60;
        telemetry.addData("Shooter RPM: ", RPM);
        telemetry.update();
    }
}