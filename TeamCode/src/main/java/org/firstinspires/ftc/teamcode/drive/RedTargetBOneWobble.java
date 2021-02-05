package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static java.lang.Math.*;

@Autonomous
public class RedTargetBOneWobble extends LinearOpMode {
    HardwarePushbotAutonomous robot = new HardwarePushbotAutonomous();   // Use a Pushbot's hardware
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-62,-58, Math.toRadians(180));
        robot.init(hardwareMap);
        drive.setPoseEstimate(startPose);

        if (robot.voltSensor.getVoltage()>= 14) {
            robot.MsWobbleAutonomous = 900;
        } else if(robot.voltSensor.getVoltage()<14 && robot.voltSensor.getVoltage()>=13){
            robot.MsWobbleAutonomous = 1000;
        } else if (robot.voltSensor.getVoltage()<13 && robot.voltSensor.getVoltage()>=12.5) {
            robot.MsWobbleAutonomous = 1060;
        } else {
            robot.MsWobbleAutonomous = 1100;
        }
        Trajectory targetZoneB = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(20, -40), Math.toRadians(0))
                .build();

        Trajectory shortForwardB = drive.trajectoryBuilder(targetZoneB.end(), true)
                .splineTo(new Vector2d(25, -40), Math.toRadians(0))
                .build();

        Trajectory ParkB = drive.trajectoryBuilder(shortForwardB.end())
                .splineTo(new Vector2d(8, -40), Math.toRadians(180))
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

//        telemetry.addData("Position: ", robot.WobbleArm.getCurrentPosition());
//        telemetry.update();
//        drive.followTrajectory(targetZoneB);
//        sleep(1000);
        //Deploy Wobble Goal by setting servo to open
    WobbleMove(-0.15); //extend
    sleep(robot.MsWobbleAutonomous);
//        WobbleMove(0);
//        sleep(300);
//        telemetry.addData("Position: ", robot.WobbleArm.getCurrentPosition());
//        telemetry.update();
//        robot.wobbleServo.setPosition(1); // or whatever is open
//        sleep(500);
//        drive.followTrajectory(shortForwardB);
//        sleep(100);
//        drive.followTrajectory(ParkB);
//        WobbleMove(0.6); //close
//        sleep(1400);

        //Deploy Arm
        //Park

//        drive.followTrajectory(aToGoal);
        //Grab Goal by setting servo to close
        //drive.followTrajectory(goalToA);
        //Release Goal by setting servo to open

    }

    public void WobbleMove(double speed){
        robot.WobbleArm.setPower(speed);
    }
}