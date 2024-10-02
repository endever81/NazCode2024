package org.firstinspires.ftc.teamcode.OldCode;

/*
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Autonomous(name = "Blue Left", group = "Automonous")

public class Old_RR_16617_2324 extends LinearOpMode {
    //-----------------------------------------------------------
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "model_23-24.tflite";  //TeamCode\build\intermediates\assets\debug  in program view
    private static final String[] LABELS = {
            "TSE"
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    HardwareRobot robot = new HardwareRobot(); //initialize RR hardware and software

//________________________________________________________

    @Override
    public void runOpMode() {
        //-----------------------------------------------------------

        robot.init(hardwareMap);  //initialize our hardwaremap
        //-----------------------------------------------------------
        initTfod();


        //-----------------------------------------------------------
        //Map All Paths for RoadRunner during initialization
        //All wheel based motion is defined here but is executed below.
        //Make changes here to change where the robot moves to.

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Red Right starting position - Same for all paths
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Approach Spike Line
        Trajectory spikeRight = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d (-33, -32, Math.toRadians(0)))
                .build();
        Trajectory spikeCenter = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d (-38, -33, Math.toRadians(90)))
                .build();
        Trajectory spikeLeft = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d (-35, -41, Math.toRadians(135)))
                .build();

        //back away from dropped pixel and spike lines
        Trajectory backFromPixelRight = drive.trajectoryBuilder(spikeRight.end())
                .lineToLinearHeading(new Pose2d (-36, -55, Math.toRadians(50)))
                .build();
        Trajectory backFromPixelCenter = drive.trajectoryBuilder(spikeCenter.end())
                .lineToLinearHeading(new Pose2d (-36, -55, Math.toRadians(95)))
                .build();
        Trajectory backFromPixelLeft = drive.trajectoryBuilder(spikeLeft.end())
                .lineToLinearHeading(new Pose2d (-36, -55, Math.toRadians(135)))
                .build();

        //Approach Backdrop
        Trajectory backDropRight = drive.trajectoryBuilder(backFromPixelRight.end())
                .lineToLinearHeading(new Pose2d (-71.25, -25, Math.toRadians(180)))
                .build();
        Trajectory backDropCenter = drive.trajectoryBuilder(backFromPixelCenter.end())
                .lineToLinearHeading(new Pose2d (-70.75, -31, Math.toRadians(180)))
                .build();
        Trajectory backDropLeft = drive.trajectoryBuilder(backFromPixelLeft.end())
                .lineToLinearHeading(new Pose2d (-71.5, -37.75, Math.toRadians(180)))
                .build();

        //Back from Backdrop
        Trajectory backUpRight = drive.trajectoryBuilder(backDropRight.end())
                .lineToLinearHeading(new Pose2d (-70.25, -25, Math.toRadians(180)))
                .build();
        Trajectory backUpCenter = drive.trajectoryBuilder(backDropCenter.end())
                .lineToLinearHeading(new Pose2d (-70, -31, Math.toRadians(180)))
                .build();
        Trajectory backUpLeft = drive.trajectoryBuilder(backDropLeft.end())
                .lineToLinearHeading(new Pose2d (-70.5, -37.75, Math.toRadians(180)))
                .build();

        //Approach Backdrop2
        Trajectory backDropRight2 = drive.trajectoryBuilder(backUpRight.end())
                .lineToLinearHeading(new Pose2d (-71.25, -25, Math.toRadians(180)))
                .build();
        Trajectory backDropCenter2 = drive.trajectoryBuilder(backUpCenter.end())
                .lineToLinearHeading(new Pose2d (-70.75, -31, Math.toRadians(180)))
                .build();
        Trajectory backDropLeft2 = drive.trajectoryBuilder(backUpLeft.end())
                .lineToLinearHeading(new Pose2d (-71, -37.5, Math.toRadians(180)))
                .build();

        //Back from Backdrop 2
        Trajectory backUpRight2 = drive.trajectoryBuilder(backDropRight2.end())
                .lineToLinearHeading(new Pose2d (-60, -26.5, Math.toRadians(180)))
                .build();
        Trajectory backUpCenter2 = drive.trajectoryBuilder(backDropCenter2.end())
                .lineToLinearHeading(new Pose2d (-60, -31, Math.toRadians(180)))
                .build();
        Trajectory backUpLeft2 = drive.trajectoryBuilder(backDropLeft2.end())
                .lineToLinearHeading(new Pose2d (-60, -37.75, Math.toRadians(180)))
                .build();

        //Move to corner and park
        Trajectory toCornerRight = drive.trajectoryBuilder(backUpRight2.end())
                .lineToLinearHeading(new Pose2d (-75, -58, Math.toRadians(0))) //Alternate end point center field x-75 y-10
                .build();
        Trajectory toCornerCenter = drive.trajectoryBuilder(backUpCenter2.end())
                .lineToLinearHeading(new Pose2d (-75, -58, Math.toRadians(0)))
                .build();
        Trajectory toCornerLeft = drive.trajectoryBuilder(backUpLeft2.end())
                .lineToLinearHeading(new Pose2d (-75, -10, Math.toRadians(0)))
                .build();

                //.strafeRight(1)
                //.forward(18)
                //.back(4)
                //.addDisplacementMarker(() -> {
                //     lift(1, 20.5);
                //       })
                //.build();

       // Trajectory toStack1 = drive.trajectoryBuilder(positiontoLine1.end())
              //  .addSpatialMarker(new Vector2d(13.75, -10), () -> {
             //       robot.servorelease.setPosition(.5);
            //         })
              //  .lineToLinearHeading(new Pose2d(13.75, -9, Math.toRadians(177)))
               // .build();



        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);

        telemetry.addData("Initiliazation Complete", "waiting for start");
        telemetry.update();

        //---------------------------------------- START COMMAND----------------------------
        waitForStart();
        //---------------------------------------- START COMMAND----------------------------
        if (isStopRequested())  return;

        //Identify which signal is being displayed.

        int x = 0;
        if (opModeIsActive()) {
            //  while (opModeIsActive()) {
            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    x = 0; //TSE on Right Spike Tape

                    for (Recognition recognition : updatedRecognitions) {
                        double xPos = recognition.getLeft();

                        if (xPos <= 200) {
                            x = 1; // TSE on Left Spike Tape
                        }
                        if (xPos > 200) {
                            x = 2; // TSE on Center Spike Tape
                        }

                    }
                }
                telemetry.update();
            }
        }
        //----------------------------------------Trajectory Execution---------------------------
        // this section executes RR paths that were built above.
        // here we can inject servo and lift commands.

        // Right Spike Tape Trajectory Path
        if (x == 0) {
            telemetry.addData("Right Spike", 10);
            telemetry.update();
            drive.followTrajectory(spikeRight);

            robot.servoDropper.setPosition(.0); // open
            sleep(1000);


            drive.followTrajectory(backFromPixelRight);
            robot.servoDropper.setPosition(.58); // close
            sleep(1000);
            robot.intake.setPower(.9);

            lift(1, 7);
            drive.followTrajectory(backDropRight);
            robot.intake.setPower(0);                      ////NEW LINE
            robot.servoDropper.setPosition(.0); // open
            sleep(1000);

            drive.followTrajectory(backUpRight);
            drive.followTrajectory(backDropRight2);
            drive.followTrajectory(backUpRight2);
            lift(1, -7);
            robot.servoDropper.setPosition(.58); // close

            drive.followTrajectory(toCornerRight);

//            drive.turn(Math.toRadians(0));
        }

        //Left Spike Tape
        if (x == 1) {
            telemetry.addData("Left Spike", 10);
            telemetry.update();
            //sleep(3000);

            drive.followTrajectory(spikeLeft);
            robot.servoDropper.setPosition(.0); // open
            sleep(1000);

            drive.followTrajectory(backFromPixelLeft);
            robot.servoDropper.setPosition(.58); // close
            sleep(1000);
            robot.intake.setPower(.9);


            lift(.5, 7);
            drive.followTrajectory(backDropLeft);
            robot.servoDropper.setPosition(.0); // open
            sleep(1000);
            robot.intake.setPower(0);                      ////NEW LINE
            drive.followTrajectory(backUpLeft);
            drive.followTrajectory(backDropLeft2);
            drive.followTrajectory(backUpLeft2);
            lift(1, -7);
            robot.servoDropper.setPosition(.58); // close

            drive.followTrajectory(toCornerLeft);

//            drive.turn(Math.toRadians(0));
        }

        // Center Spike Tape
        if (x == 2) {
            telemetry.addData("Center Spike", 10);
            telemetry.update();
            //sleep(3000);
            drive.followTrajectory(spikeCenter);

            robot.servoDropper.setPosition(.0); // open
            sleep(1000);
            drive.followTrajectory(backFromPixelCenter);
            robot.servoDropper.setPosition(.58); // close
            sleep(2000);
            robot.intake.setPower(.9);

            lift(.5, 7);
            drive.followTrajectory(backDropCenter);
            robot.servoDropper.setPosition(.0); // open
            robot.intake.setPower(0);                      ////NEW LINE
            sleep(1000);
            drive.followTrajectory(backUpCenter);
            drive.followTrajectory(backDropCenter2);
            drive.followTrajectory(backUpCenter2);
            lift(1, -7);
            robot.servoDropper.setPosition(.58); // close
            drive.followTrajectory(toCornerRight);


//            drive.turn(Math.toRadians(0));
        }


    }

    public void lift(double power, double inches)
    {
        int newLiftTargetRight;
        int newLiftTargetLeft;


        if (opModeIsActive()) {


            newLiftTargetLeft = robot.liftleft.getCurrentPosition() - (int) (inches * (1140 / (3.5 * 3.1415)));
            newLiftTargetRight = robot.liftright.getCurrentPosition() - (int) (inches * (1140 / (3.5 * 3.1415)));

            robot.liftleft.setTargetPosition(newLiftTargetLeft);
            robot.liftright.setTargetPosition(newLiftTargetRight);

            robot.liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.liftleft.setPower(Math.abs(power));
            robot.liftright.setPower(Math.abs(power));


        }

    }

// Initialize the TensorFlow Object Detection processor.

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()


    // * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}


*/