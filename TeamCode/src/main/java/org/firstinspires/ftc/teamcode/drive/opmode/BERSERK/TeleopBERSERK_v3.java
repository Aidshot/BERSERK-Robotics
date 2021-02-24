package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;


//Teleop utilizes Align to point and Shooter Velocity PID Control

//@Config
@TeleOp(group = "BERSERK")
public class TeleopBERSERK_v3 extends LinearOpMode {

    //public static double DRAWING_TARGET_RADIUS = 2;
    public static double DRAWING_TARGET_RADIUS = 1;

    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    //Target Position of TowerGoal
    private Vector2d targetPosition = new Vector2d(72, 36);

    //Velocity PID
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.003, 0, 0);
    public static double kV = 0.00039;
    public static double kA = 0.00015;
    public static double kStatic = 0;
    private double lastTargetVelo = 0.0;
    private final PIDFController veloController = new PIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    //Timer
    private final ElapsedTime veloTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Initializations
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot       = new HardwareBERSERK();
        robot.init(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // Motor Setup //
        DcMotorEx myMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx myMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Start Pose from PoseStorage
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        //Start Pose from left starting position
        //drive.getLocalizer().setPoseEstimate(new Pose2d(-63,50));

        headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        ////VARIABLES\\\\\

        //Flap
        double launch_angle = 0.174;
        double launch_angle_offset = 0;
        double max_launch_angle = 0.2;
        double min_launch_angle = 0.113;

        //Kicker
        double kicker_out = 0.68;
        double kicker_in = 0.2;

        //Wobble Claw
        double wobble_close = 0.18;
        double wobble_open = 0.6;

        //Wobble Lift
        double wobble_up = 0.6;
        double wobble_down = 0.2;

        //Initial Shooter Velocity
        double targetVelo = 0;

        //Miscellaneous
        long shootWait = 150;
        double turn_right = -7;
        double turn_left = 12;

        //Set Servos
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_open);
        robot.kicker.setPosition(kicker_out);
        //robot.emergency_servo.setPosition(emergency_open);

        if (isStopRequested()) return;

        // Start the veloTimer
        veloTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            //Gamepad Input
            double ly = -gamepad1.left_stick_y;
            double lx = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            //Initialize Localizer
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            Pose2d driveDirection = new Pose2d();

            //Initialize FTC Dashboard
            Canvas fieldOverlay = packet.fieldOverlay();
            //TelemetryPacket packet = new TelemetryPacket();

            // Call necessary controller methods
            veloController.setTargetPosition(targetVelo);
            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();

            lastTargetVelo = targetVelo;

            // Get the velocity from the motor with the encoder
            double motorVelo = myMotor1.getVelocity();

            // Update the controller and set the power for each motor
            double power = veloController.update(motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);

            //// Gamepad Controls \\\\

            //Intake + Indexer
            if (gamepad1.right_bumper){
                robot.intake.setPower(1);
                robot.feeder_turn.setPower(1);
            }
            else if (gamepad1.left_bumper){
                robot.intake.setPower(0);
                robot.feeder_turn.setPower(0);
            }

            //Gamepad 2 Right Bumper reverses intake
            if (gamepad2.right_bumper){
                robot.intake.setPower(-1);
                robot.feeder_turn.setPower(-1);
            }

            //Shooter
            if (gamepad1.a || gamepad2.a) {
                targetVelo = 1700; //1800
            }
            else if (gamepad1.b || gamepad2.b) {
                targetVelo = 0;
            }

            //Distance to Tower
            double getDistance = Math.sqrt(Math.pow(targetPosition.getX() - poseEstimate.getX(),2) + Math.pow(targetPosition.getY() - poseEstimate.getY(),2));

            //Automatic Flap Adjustment
            if (90 >= getDistance && getDistance>= 50) {
                launch_angle = 0.174;
            }
            else if (110 >= getDistance && getDistance>= 90) {
                launch_angle = 0.172;
            }
            else if (130 >= getDistance && getDistance>= 110) {
                launch_angle = 0.170;
            }
            else if (140 >= getDistance && getDistance>= 130) {
                launch_angle = 0.166;
            }
            else if (150 >= getDistance && getDistance>= 140) {
                launch_angle = 0.160;
            }

            //Backwall= 133 in
            //Shooting Spot= 74 in

            //Flap Set Position
            robot.flap.setPosition(Math.min(Math.max(launch_angle + launch_angle_offset, min_launch_angle), max_launch_angle));

            //Flap Manual Offset
            if (gamepad1.dpad_up) {
                launch_angle_offset += -0.00025;
            }
            else if (gamepad1.dpad_down) {
                launch_angle_offset += 0.00025;
            }

            // Y prepares for endgame by dropping flap and powering up shooter
            if (gamepad1.y) {
                targetVelo = 1700;
                launch_angle_offset = 0.2;
            }

            //Powershot Turns
            if (gamepad1.dpad_left) {
                drive.turn(Math.toRadians(turn_left));
            }
            else if (gamepad1.dpad_right) {
                drive.turn(Math.toRadians(turn_right));
            }

            //Wobble Arm
            if (gamepad2.dpad_up) {
                robot.wobble_lift.setPosition(wobble_up);
            }
            else if (gamepad2.dpad_down) {
                robot.wobble_lift.setPosition(wobble_down);
            }
            else if (gamepad2.dpad_left) {
                robot.wobble_claw.setPosition(wobble_open);
            }
            else if (gamepad2.dpad_right) {
                robot.wobble_claw.setPosition(wobble_close);
            }

            //Feeder Servo
            if (gamepad1.x) {
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);
            }

            //Fold-Out Lift
            if (gamepad2.right_stick_y < 0) {
                robot.foldout_lift.setPower(1);
            }
            else if (gamepad2.right_stick_y > 0) {
                robot.foldout_lift.setPower(-1);
            }
            else robot.foldout_lift.setPower(0);

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch to align to point if gamepad1 left trigger is activated
                    if (Math.abs(gamepad1.left_trigger) >= 0.5) {
                        currentMode = Mode.ALIGN_TO_POINT;
                    }

                    driveDirection = new Pose2d(
                            Math.signum(ly) * ly * ly,
                            Math.signum(lx) * lx * lx,
                            Math.signum(rx) * rx * rx

                            //-gamepad1.left_stick_y,
                            //-gamepad1.left_stick_x,
                            //-gamepad1.right_stick_x
                    );
                    break;

                case ALIGN_TO_POINT:
                    // Switch to normal control if gamepad1 right trigger is activated
                    if (Math.abs(gamepad1.right_trigger) >= 0.5) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    Vector2d fieldFrameInput = new Vector2d(
                            Math.signum(ly) * ly * ly,
                            Math.signum(lx) * lx * lx

                            //-gamepad1.left_stick_y,
                            //-gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    double theta = difference.angle();
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());
                    headingController.setTargetPosition(theta);

                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    //Dashboard Setup
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                    break;
            }

            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
            drive.setWeightedDrivePower(driveDirection);
            headingController.update(poseEstimate.getHeading());
            drive.getLocalizer().update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            //DS Telemetry
            telemetry.addData("Distance", getDistance);
            telemetry.addData("launch angle", launch_angle+launch_angle_offset);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("mode", currentMode);
            telemetry.update();

            //Dashboard Telemetry
            packet.put("FlyWheel Velocity", motorVelo);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}