package sample_camera_opmodes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import for_camera_opmodes.OpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "DetectColor", group = "ZZOpModeCameraPackage")
@Disabled

public class DetectColor extends OpModeCamera {

  int ds2 = 8;  // additional downsampling of the image
  private int looped = 0;
  private long lastLoopTime = 0;
  // set to 1 to disable further downsampling

  /*
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */
  @Override
  public void init() {
    setCameraDownsampling(1);
    // parameter determines how downsampled you want your images
    // 8, 4, 2, or 1.
    // higher number is more downsampled, so less resolution but faster
    // 1 is original resolution, which is detailed but slow
    // must be called before super.init sets up the camera

    super.init(); // inits camera functions, starts preview callback
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    long startTime = System.currentTimeMillis();

    String colorString = getColorFromCamera(telemetry);
    telemetry.addData("Color:", "Color detected is: " + colorString);

    long endTime = System.currentTimeMillis();
    telemetry.addData("Dims", Integer.toString(width / ds2) + " x " + Integer.toString(height / ds2));
    telemetry.addData("Loop Time", Long.toString(endTime - startTime));
    telemetry.addData("Loop to Loop Time", Long.toString(endTime - lastLoopTime));

    lastLoopTime = endTime;
  }

  public String getColorFromCamera(Telemetry atelemetry) {

    String colorString = "N/A";

    if (imageReady()) { // only do this if an image has been returned from the camera
      int redValue = 0;
      int blueValue = 0;
      int greenValue = 0;

      // get image, rotated so (0,0) is in the bottom left of the preview window
      Bitmap rgbImage;
      rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

      telemetry.addData("width", "%d", width);
      telemetry.addData("height", "%d", height);
      telemetry.addData(" rgbImage.getWidth()", "%d",  rgbImage.getWidth());
      telemetry.addData(" rgbImage.getHeight()", "%d", rgbImage.getHeight());

      for (int x = 25; x < rgbImage.getWidth() - 50; x++) {
        for (int y = 70; y < rgbImage.getHeight() - 10; y++) {
          int pixel = rgbImage.getPixel(x, y);
          redValue += red(pixel);
          blueValue += blue(pixel);
          greenValue += green(pixel);
        }
      }

      telemetry.addData("redValue", "%d", redValue);
      telemetry.addData("blueValue", "%d", blueValue);
      telemetry.addData("greenValue", "%d", greenValue);

      int color = highestColor(redValue, greenValue, blueValue);

      switch (color) {
        case 0:
          colorString = "RED";
          break;
        case 1:
          colorString = "GREEN";
          break;
        case 2:
          colorString = "BLUE";
      }

    }
    return(colorString);
  }

  @Override
  public void stop() {
    super.stop(); // stops camera functions
  }
}
