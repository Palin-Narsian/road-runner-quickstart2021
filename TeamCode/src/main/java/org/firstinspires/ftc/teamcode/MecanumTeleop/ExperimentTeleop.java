package org.firstinspires.ftc.teamcode.MecanumTeleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumTeleop.HardwarePushbot;

import static java.lang.Thread.sleep;


/**
 * Created by robot on 9/17/2018.
 */
@TeleOp(name = " Experiment Teleop")

public class ExperimentTeleop extends OpMode
{

    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    double WobblePositionOpen; //0=fully in 1= fully extended 2= halfway up 3= extended in consideration for wall
    double engame = 0;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void init()  {
        robot.init(hardwareMap);
        robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initGyro();
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


        if (gamepad2.x) {
            robot.shooter.setVelocity(0);
        }
        if (gamepad2.right_bumper) {
            robot.shooter.setVelocity(1910);
        }

        if (gamepad2.left_bumper) {
            robot.shooter.setVelocity(720);
        }

        if (gamepad1.a) {
            robot.test.setPosition(0.4);
            try {
                sleep(500);
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
        // robot.shooter.setVelocity(robot.shooterVar);


        if (gamepad1.x) {
            engame = 2;
        }
        if (engame >0) {
            if (engame == 2) {
                robot.test.setPosition(0.4);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.test.setPosition(0);
                try {
                    sleep(750);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            try {
                EndgamePowershot();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            engame = engame - 1;
        }




        if (gamepad2.b) {
            robot.wobbleServo.setPosition(1);
        }

        if (gamepad2.a) {
            robot.wobbleServo.setPosition(0);
        }

        robot.WobbleArm.setPower(gamepad2.left_stick_y*0.25
        );

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
    public void EndgamePowershot() throws InterruptedException {


    turnWithGyro(13, 0.3);
    sleep(500);
    robot.test.setPosition(0.4);
        sleep(500);
        robot.test.setPosition(0);
        sleep(750);



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
    public void turn(double timeoutS) {
        int gain = 80;
        int currentPositionlf = robot.lf.getCurrentPosition();
        int currentPositionrf = robot.rf.getCurrentPosition();
        int currentPositionlb = robot.lb.getCurrentPosition();
        int currentPositionrb = robot.rb.getCurrentPosition();

        robot.lf.setTargetPosition(currentPositionlf+gain);
        robot.rf.setTargetPosition(currentPositionrf-gain);
        robot.lb.setTargetPosition(currentPositionlb+gain);
        robot.rb.setTargetPosition(currentPositionrb-gain);
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lf.setPower(0.2);
        robot.rf.setPower(0.2);
        robot.lb.setPower(0.2);
        robot.rb.setPower(0.2);

        while ( (runtime.seconds() < timeoutS)&&(robot.lf.isBusy() && robot.rf.isBusy()&&robot.lb.isBusy()&&robot.rb.isBusy())) {

        }
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 20){
                first = (degrees - 20) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 20){
                first = devertify(-(degrees - 20) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb)) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb))) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb)) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
        }else {
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb))) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
        }
        //</editor-fold>
        //
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    public void turnWithEncoder(double input){
        robot.lf.setMode(DcMotor. RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        robot.lf.setPower(input);
        robot.rf.setPower(-input);
        robot.lb.setPower(input);
        robot.rb.setPower(-input);
    }
}