package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

import static java.lang.Math.*;

@Autonomous
public class

RedShoot extends LinearOpMode {
    HardwarePushbotAutonomous robot = new HardwarePushbotAutonomous();   // Use a Pushbot's hardware
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-62,-58, Math.toRadians(180));
        robot.init(hardwareMap);
        drive.setPoseEstimate(startPose);

        if (robot.voltSensor.getVoltage()>= 14) {
            robot.MsWobbleAutonomous = 1000;
        } else if(robot.voltSensor.getVoltage()<14 && robot.voltSensor.getVoltage()>=13.5){
            robot.MsWobbleAutonomous = 1100;
        } else if (robot.voltSensor.getVoltage()<13.5 && robot.voltSensor.getVoltage()>=13){
            robot.MsWobbleAutonomous = 1300;
        } else if (robot.voltSensor.getVoltage()<13 && robot.voltSensor.getVoltage()>=12.5) {
            robot.MsWobbleAutonomous = 1360;
        } else if (robot.voltSensor.getVoltage()<12.5){
            robot.MsWobbleAutonomous = 1400;
        }

        Trajectory moveForward = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-5, -58), Math.toRadians(0))
                .build();
        Trajectory strafe = drive.trajectoryBuilder(moveForward.end(),true)
                .lineToLinearHeading(new Pose2d(-5, -24, Math.toRadians(173)))
                .build();
//        Trajectory aToGoal = drive.trajectoryBuilder(targetZoneA.end(),true)
//                .splineTo(new Vector2d(-24,31), Math.toRadians(180))
//                .build();
//
//        Trajectory goalToA = drive.trajectoryBuilder(aToGoal.end())
//                .splineTo(new Vector2d(10,50), Math.toRadians(0))
//                .build();



        waitForStart();

        if(isStopRequested()) return;

//

        robot.shooter.setVelocity(1910);
        drive.followTrajectory(moveForward);
//        sleep(100);
        telemetry.addData("position", robot.WobbleArm.getCurrentPosition());
        telemetry.update();
        drive.followTrajectory(strafe);
        sleep(2000);
        robot.shoot();
        sleep(300);
        robot.shoot();
        sleep(300);
        robot.shoot();
        sleep(300);

        //Deploy Arm
        //Park

//

    }

    public void WobbleMove(double speed){
        robot.WobbleArm.setPower(speed);
    }
}