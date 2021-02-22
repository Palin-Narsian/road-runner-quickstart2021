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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumTeleop.HardwarePushbot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.OpenCVTest;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

import static java.lang.Math.*;

@Autonomous
public class RedTwoWobble extends LinearOpMode {
    HardwarePushbotAutonomous robot = new HardwarePushbotAutonomous();   // Use a Pushbot's hardware
    OpenCvWebcam webcam;
    OpenCVTest.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());        pipeline = new OpenCVTest.SkystoneDeterminationPipeline();
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

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
        Trajectory targetZoneA = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-5, -58), Math.toRadians(0))
                .build();

        Trajectory lineUpAOne= drive.trajectoryBuilder(targetZoneA.end())
                .lineToLinearHeading(new Pose2d(-5, -8, Math.toRadians(0)))
                .build();
        Trajectory lineUpATwo= drive.trajectoryBuilder(lineUpAOne.end(), true)
                .splineTo(new Vector2d(-20, -8), Math.toRadians(198.5))
                .build();
        Trajectory goForSecondWobbleA = drive.trajectoryBuilder(lineUpATwo.end())
                .lineToConstantHeading(new Vector2d(-35.8, -25.9), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(9, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory lineUpToDropSecondWobbleA = drive.trajectoryBuilder(goForSecondWobbleA.end())
                .splineTo(new Vector2d(-20, -8), Math.toRadians(170))
                .build();
        Trajectory secondTargetZoneA= drive.trajectoryBuilder(lineUpToDropSecondWobbleA.end(), true)
                .lineToConstantHeading(new Vector2d(-5, -53))
                .build();
        Trajectory backUpA= drive.trajectoryBuilder(secondTargetZoneA.end())
                .lineToConstantHeading(new Vector2d(-15, -53))
                .build();
        Trajectory parkSecondA= drive.trajectoryBuilder(backUpA.end())
                .lineToLinearHeading(new Pose2d(8, -30, Math.toRadians(180)))
                .build();
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
                                        new MecanumVelocityConstraint(9, DriveConstants.TRACK_WIDTH)
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
        Trajectory targetZoneC = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(42, -60), Math.toRadians(0))
                .build();

        Trajectory lineUpCOne= drive.trajectoryBuilder(targetZoneC.end())
                .splineTo(new Vector2d(10, -15), Math.toRadians(0))
                .build();
        Trajectory lineUpCTwo= drive.trajectoryBuilder(lineUpCOne.end(),true)
                .splineTo(new Vector2d(-20, -8), Math.toRadians(199))
                .build();
        Trajectory goForSecondWobbleC = drive.trajectoryBuilder(lineUpCTwo.end())
                .lineToConstantHeading(new Vector2d(-35.8, -25.9), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(9, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory lineUpToDropSecondWobbleC = drive.trajectoryBuilder(goForSecondWobbleC.end())
                .splineTo(new Vector2d(-20, -8), Math.toRadians(170))
                .build();
        Trajectory secondTargetZoneC= drive.trajectoryBuilder(lineUpToDropSecondWobbleC.end(), true)
                .splineTo(new Vector2d(42, -53), Math.toRadians(-10))
                .build();
        Trajectory parkSecondC= drive.trajectoryBuilder(secondTargetZoneC.end())
                .splineTo(new Vector2d(8, -50), Math.toRadians(180))
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


        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        sleep(800);

        if (pipeline.position == OpenCVTest.SkystoneDeterminationPipeline.RingPosition.FOUR) {
            telemetry.addData("Target C", 1);
            drive.followTrajectory(targetZoneC);
            sleep(200);
            //Deploy Wobble Goal by setting servo to open
            WobbleMove(0.15); //extend
            sleep(robot.MsWobbleAutonomous);
            WobbleMove(0);
            sleep(300);
            robot.wobbleServo.setPosition(0); // or whatever is open
            sleep(500);

            drive.followTrajectory(lineUpCOne);
            sleep(100);
            drive.followTrajectory(lineUpCTwo);
            sleep(100);
            drive.followTrajectory(goForSecondWobbleC);
            sleep(700);
            robot.wobbleServo.setPosition(1); // or whatever is open
            sleep(600);
            drive.followTrajectory(lineUpToDropSecondWobbleC);
            sleep(100);
            drive.followTrajectory(secondTargetZoneC);
            sleep(100);
            robot.wobbleServo.setPosition(0); // or whatever is open
            sleep(600);
            drive.followTrajectory(parkSecondC);
            sleep(100);

        } else if (pipeline.position == OpenCVTest.SkystoneDeterminationPipeline.RingPosition.ONE) {
            telemetry.addData("Target B", 1);
            drive.followTrajectory(targetZoneB);
            sleep(200);
            //Deploy Wobble Goal by setting servo to open
            WobbleMove(0.15); //extend
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
        } else {
            telemetry.addData("Target A", 1);
            drive.followTrajectory(targetZoneA);
            sleep(200);
            //Deploy Wobble Goal by setting servo to open
            WobbleMove(0.15); //extend
            sleep(robot.MsWobbleAutonomous);
            WobbleMove(0);
            sleep(300);
            robot.wobbleServo.setPosition(0); // or whatever is open
            sleep(500);

            drive.followTrajectory(lineUpAOne);
            sleep(100);
            drive.followTrajectory(lineUpATwo);
            sleep(100);
            drive.followTrajectory(goForSecondWobbleA);
            sleep(700);
            robot.wobbleServo.setPosition(1); // or whatever is open
            sleep(600);
            drive.followTrajectory(lineUpToDropSecondWobbleA);
            sleep(100);
            drive.followTrajectory(secondTargetZoneA);
            sleep(100);
            robot.wobbleServo.setPosition(0); // or whatever is open
            sleep(600);
            drive.followTrajectory(backUpA);
            sleep(100);
            drive.followTrajectory(parkSecondA);
            sleep(100);
        }

        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);
    }


    //Deploy Arm
    //Park

//        drive.followTrajectory(aToGoal);
    //Grab Goal by setting servo to close
    //drive.followTrajectory(goalToA);
    //Release Goal by setting servo to open

    public void WobbleMove(double speed){
        robot.WobbleArm.setPower(speed);
    }
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(165,130);

        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 30;

        final int FOUR_RING_THRESHOLD = 151;
        final int ONE_RING_THRESHOLD = 132;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile OpenCVTest.SkystoneDeterminationPipeline.RingPosition position = OpenCVTest.SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = OpenCVTest.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = OpenCVTest.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = OpenCVTest.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = OpenCVTest.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}