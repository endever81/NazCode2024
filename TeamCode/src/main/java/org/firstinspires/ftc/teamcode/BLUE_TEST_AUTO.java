package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Config
@Autonomous(name = "BLUE_TEST_AUTO", group = "Autonomous")
public class BLUE_TEST_AUTO extends LinearOpMode {
    public class Lift {
        private DcMotorEx liftV;

        public Lift(HardwareMap hardwareMap) {
            liftV = hardwareMap.get(DcMotorEx.class, "liftV");
            liftV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftV.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftV.setPower(0.8);
                    initialized = true;
                }

                double pos = liftV.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 500.0) {
                    return true;
                } else {
                    liftV.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftV.setPower(-0.8);
                    initialized = true;
                }

                double pos = liftV.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0.0) {
                    return true;
                } else {
                    liftV.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Extender {
        private DcMotorEx extendH;

        public Extender (HardwareMap hardwareMap) {
            extendH = hardwareMap.get(DcMotorEx.class, "liftH");
            extendH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendH.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendH.setPower(0.8);
                    initialized = true;
                }

                double pos = extendH.getCurrentPosition();
                packet.put("extPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    extendH.setPower(0);
                    return false;
                }
            }
        }
        public Action liftOut() {
            return new LiftOut();
        }

        public class LiftIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendH.setPower(-0.8);
                    initialized = true;
                }

                double pos = extendH.getCurrentPosition();
                packet.put("extPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    extendH.setPower(0);
                    return false;
                }
            }
        }
        public Action liftIn(){
            return new LiftIn();
        }
    }

    public static class Rotate {
        private Servo rotate;

        public Rotate(HardwareMap hardwareMap) {
            rotate = hardwareMap.get(Servo.class, "servo_rotate");
        }

        public class RotateUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(.4);
                return false;
            }
        }
        public Action rotateUp() {
            return new RotateUp();
        }

        public class RotateDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(0.4);
                return false;
            }
        }
        public Action rotateDown() {
            return new RotateDown();
        }
    }

    HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Rotate rotate = new Rotate(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Extender extender = new Extender(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //.lineToYSplineHeading(5, Math.toRadians(90))
               // .waitSeconds(2)
                //.setTangent(Math.toRadians(2))
                //.lineToY(5)
                //.setTangent(Math.toRadians(0))
                //.lineToX(5)
                .strafeToLinearHeading(new Vector2d(39, 0),Math.toRadians(90))
                .strafeTo(new Vector2d(39, 0));
                //.turn(Math.toRadians(180))
                //.lineToY(5)
                //.waitSeconds(.1);
/*
        Pose2d tab1Pose = new Pose2d(39, 0, Math.toRadians(90));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(tab1Pose)
                .splineTo(new Vector2d(0.0, 48.0), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(35,37, 0.0), Math.toRadians(180))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);

       */
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(rotate.rotateUp());



        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        lift (1, 20);

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                        //lift.liftUp(),
                      //  rotate.rotateDown()
                        //lift.liftDown(),
                        //trajectoryActionCloseOut
                )
        );
        lift (1, -10);

        sleep(500);
    }
    public void lift(double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;

        if (opModeIsActive()) {

            newLiftTarget = robot.liftV.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415)));

            robot.liftV.setTargetPosition(newLiftTarget);

            robot.liftV.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftV.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftV.isBusy()) {
                       telemetry.addData("Lift", "Running at %7d",
                                       robot.liftV.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftV.setPower(0);
          robot.liftV.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

          }
     }


}

