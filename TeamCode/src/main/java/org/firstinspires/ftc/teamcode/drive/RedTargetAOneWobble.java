package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumTeleop.HardwarePushbot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static java.lang.Math.*;

@Autonomous
@Disabled
public class RedTargetAOneWobble extends LinearOpMode {
    HardwarePushbotAutonomous robot = new HardwarePushbotAutonomous();   // Use a Pushbot's hardware
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-62,-58, Math.toRadians(180));
        robot.init(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory targetZoneA = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-5, -58), Math.toRadians(0))
                .build();

        Trajectory shortForward = drive.trajectoryBuilder(targetZoneA.end(), true)
                .splineTo(new Vector2d(3, -58), Math.toRadians(0))
                .build();
        Trajectory shortBackward = drive.trajectoryBuilder(shortForward.end())
                .splineTo(new Vector2d(-5, -58), Math.toRadians(0))
                .build();
        Trajectory Park = drive.trajectoryBuilder(shortBackward.end(), true)
                .splineTo(new Vector2d(9, -40), Math.toRadians(0))
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

       drive.followTrajectory(targetZoneA);
       sleep(700);
        //Deploy Wobble Goal by setting servo to open
        WobbleMove(-0.6); //extend
        sleep(1400); //1600
        WobbleMove(0);
        sleep(300);
        telemetry.addData("Position: ", robot.WobbleArm.getCurrentPosition());
        telemetry.update();
        robot.wobbleServo.setPosition(1); // or whatever is open
        sleep(500);
        drive.followTrajectory(shortForward);
        drive.followTrajectory(shortBackward);
        sleep(100);
        drive.followTrajectory(Park);
        WobbleMove(0.6); //close
        sleep(1400);


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