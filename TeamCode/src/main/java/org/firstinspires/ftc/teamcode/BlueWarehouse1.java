package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


// Name the program as it shows up in the robotr Station
@Autonomous(name = "Blue Warehouse")

public class BlueWarehouse1 extends LinearOpMode {

    SampleMecanumDrive vector;
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    // Import the variables from the util class for the motors and servos

    // Declare counters at natural starting point
    /*
    As the Op mode runs, either of the integer counters will increase based on whether an object is detected or absent.
    The variable with a higher count after 10 loops determines the likelihood of an object actually present in the webcam's view.
     */
    int objectDetected = 0;
    int objectAbsent= 0;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZngU9H/////AAABmWcUJSDyAkXTsV1ZbtiI0c6Ff6REKfqt0K5ILdhwDfDX/lDPPESZZnlQJOWzJd+q4CIJ4ExzUj5i92YJQrebNEBqaAR03Xf8OWDWgj8MDNrfa8Wu3FGtsp6fyrFUy+f7lNhfC/4TSmk1zVkFxyK34H+mW3LeeChBThKM6bvC3d7SV6bxSwcSXAfN9ZFMbxpMVCjW/J8TO/MmABRBqU27CvwM59zhvQknC3euq85hQS44i86KVggsajKQ0NEEdRjfF0WVchimOvOGXHjCVmQ2sPiXGITzcpuX92ifC8t/gJLcbQCvB6goatUrWY6AB9VYJIQpSXKD97/nmAO/nCVIHJ/mIZ6Ed82hR4kwNXIULe+A";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        vector = new SampleMecanumDrive(hardwareMap);
        initVuforia();
        initTfod();
        //robot inputs all defined variables from the SampleMecanumrobot class

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.0, 1.0);

            // Only a single object should be visible in the webcam's view, so the viewing area is squared.
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            for(int ti = 1; ti < 20; ti++) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;

                            // check label to see if the camera now sees a Duck
                            if (recognition.getLabel().equals("Duck")) {
                                telemetry.addData("Object Detected", "Duck");
                                objectDetected++;
                            } else {
                                objectAbsent++;
                            }
                        }
                        telemetry.update();
                    }
                }
            }

            if (objectDetected > objectAbsent) {
                // The object is located on the right-most spot
                telemetry.addData("Position: ", "Middle");
                telemetry.update();
                tfod.shutdown();
                sleep(500);
                objectMiddle();
            }
            else  {
                // The object is not on the right-most spot, so we move left to the middle spot
                Trajectory strafeMiddle = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                        .lineToLinearHeading(new Pose2d(6, 6, Math.toRadians(0)))
                        .build();
                vector.followTrajectory(strafeMiddle);
                sleep(1000);
                for(int ti = 1; ti < 20; ti++) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());

                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                i++;

                                // check label to see if the camera now sees a Duck
                                if (recognition.getLabel().equals("Duck")) {
                                    telemetry.addData("Object Detected", "Duck");
                                    objectDetected++;
                                } else {
                                    objectAbsent++;
                                }
                            }
                            telemetry.update();
                        }
                    }
                }

                if (objectDetected > objectAbsent) {
                    // The object is detected on the middle spot
                    telemetry.addData("Position: ", "Right");
                    telemetry.update();
                    tfod.shutdown();
                    sleep(500);
                    objectRight();
                }
                else {
                    // The object was not detected in the middle, so it must be on the left spot
                    telemetry.addData("Position: ", "Left");
                    telemetry.update();
                    tfod.shutdown();
                    sleep(500);
                    objectLeft();
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    public void gyroCorrect()
    {
        vector.angles = vector.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        vector.absHeading = vector.angles.firstAngle;
        telemetry.addData("Heading", vector.absHeading);
        telemetry.update();
        for(int f = 1; f < 20; f++)
        {
            vector.leftFront.setPower(-0.10 * (vector.absHeading - vector.originalHeading));
          //  vector.leftRear.setPower(-0.05 * (vector.absHeading - vector.originalHeading));
           // vector.rightFront.setPower(0.05 * (vector.absHeading - vector.originalHeading));
            vector.rightRear.setPower(0.10 * (vector.absHeading - vector.originalHeading));
            vector.angles = vector.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            vector.absHeading = vector.angles.firstAngle;
            telemetry.addData("Heading", vector.absHeading);
            telemetry.update();
           //sleep();
        }
        vector.leftFront.setPower(0);
        vector.leftRear.setPower(0);
        vector.rightFront.setPower(0);
        vector.rightRear.setPower(0);
        telemetry.update();
    }


    public void objectRight()
    {
        Trajectory strafeTest = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(0)))
                .build();
        vector.followTrajectory(strafeTest);

        gyroCorrect();
    }
    public void objectMiddle()
    {
        Trajectory strafeTest = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(90)))
                .build();
        vector.followTrajectory(strafeTest);

        gyroCorrect();

    }
    public void objectLeft()
    {
        Trajectory strafeTest = vector.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 20, Math.toRadians(90)))
                .build();
        vector.followTrajectory(strafeTest);

        gyroCorrect();

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "frontWebcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}