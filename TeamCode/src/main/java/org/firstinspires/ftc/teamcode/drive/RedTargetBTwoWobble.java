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
public class RedTargetBTwoWobble extends LinearOpMode {
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
        Trajectory targetZoneB = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(20, -40), Math.toRadians(0))
                .build();

        Trajectory lineUpB = drive.trajectoryBuilder(targetZoneB.end())
                .splineTo(new Vector2d(-20, -8), Math.toRadians(25))
                .build();
        Trajectory goForSecondWobbleB = drive.trajectoryBuilder(lineUpB.end())
                .lineToConstantHeading(new Vector2d(-35.8, -25.9), new MinVelocityConstraint(
                Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                )
        ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory lineUpToDropSecondWobbleB = drive.trajectoryBuilder(goForSecondWobbleB.end())
                .splineTo(new Vector2d(-20, -8), Math.toRadians(170))
                .build();
        Trajectory secondTargetZoneB = drive.trajectoryBuilder(lineUpToDropSecondWobbleB.end(), true)
                .splineTo(new Vector2d(25, -30), Math.toRadians(-10))
                .build();
        Trajectory parkSecondB = drive.trajectoryBuilder(secondTargetZoneB.end())
                .splineTo(new Vector2d(8, -30), Math.toRadians(180))
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
        drive.followTrajectory(targetZoneB);
        sleep(200);
        //Deploy Wobble Goal by setting servo to open
        WobbleMove(-0.15); //extend
        sleep(robot.MsWobbleAutonomous);
        WobbleMove(0);
        sleep(300);
        robot.wobbleServo.setPosition(0); // or whatever is open
        sleep(500);
        drive.followTrajectory(lineUpB);
        sleep(100);
        drive.followTrajectory(goForSecondWobbleB);
        sleep(700);
        robot.wobbleServo.setPosition(1); // or whatever is open
        sleep(600);
        drive.followTrajectory(lineUpToDropSecondWobbleB);
        sleep(100);
        drive.followTrajectory(secondTargetZoneB);
        sleep(100);
        robot.wobbleServo.setPosition(0); // or whatever is open
        sleep(600);
        drive.followTrajectory(parkSecondB);
        sleep(100);

        //Deploy Arm
        //Park

//

    }

    public void WobbleMove(double speed){
        robot.WobbleArm.setPower(speed);
    }
}