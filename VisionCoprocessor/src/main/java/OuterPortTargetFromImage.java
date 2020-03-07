import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.LinkedList;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

//import edu.wpi.first.wpilibj.vision.VisionPipeline;
import edu.wpi.first.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import org.opencv.highgui.HighGui;
import org.opencv.core.MatOfPoint;

/**
* OuterPortTargetFromImage class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class OuterPortTargetFromImage implements VisionPipeline {

	public static class OuterPortTarget {
		public OuterPortTarget(ArrayList<MatOfPoint> target) {
			retroTarget = target;
		}
		private ArrayList<MatOfPoint> retroTarget = null;

		public void drawOn(Mat img) {
			drawOn(img, new Scalar(255,255,255));
		}
		public void drawOn(Mat img, Scalar color) {
			LinkedList<MatOfPoint> targets = new LinkedList<>();
			for (int i = 0; i < retroTarget.size(); i++) {
				targets.add(retroTarget.get(i));
			}
			Imgproc.drawContours(img, targets, -1, color);
		}
	}

	//Outputs
	private Mat blurOutput = new Mat();
	private Mat hsvThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private List<OuterPortTarget> detectedTargets = new LinkedList<>();

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	@Override	public void process(Mat source0) {
		// Step Blur0:
		Mat blurInput = source0;
		BlurType blurType = BlurType.get("Box Blur");
		double blurRadius = 3.6036036036036037;
		blur(blurInput, blurType, blurRadius, blurOutput);

		// Step HSV_Threshold0:
		Mat hsvThresholdInput = blurOutput;
		double[] hsvThresholdHue = {74.46043165467626, 80.0};
		double[] hsvThresholdSaturation = {201.79856115107913, 255.0};
		double[] hsvThresholdValue = {40, 255.0};
		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step Find_Contours0:
		Mat findContoursInput = hsvThresholdOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		detectedTargets = findTargets(findContoursOutput());

	}

	/**
	 * This method is a generated getter for the output of a Blur.
	 * @return Mat output from Blur.
	 */
	public Mat blurOutput() {
		return blurOutput;
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	public List<OuterPortTarget> getDetectedTargets() {
		return detectedTargets;
	}

	/**
	 * An indication of which type of filter to use for a blur.
	 * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
	 */
	enum BlurType{
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
			BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
		}

		public static BlurType get(String type) {
			if (BILATERAL.label.equals(type)) {
				return BILATERAL;
			}
			else if (GAUSSIAN.label.equals(type)) {
			return GAUSSIAN;
			}
			else if (MEDIAN.label.equals(type)) {
				return MEDIAN;
			}
			else {
				return BOX;
			}
		}

		@Override
		public String toString() {
			return this.label;
		}
	}

	/**
	 * Softens an image using one of several filters.
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	private void blur(Mat input, BlurType type, double doubleRadius,
		Mat output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type){
			case BOX:
				kernelSize = 2 * radius + 1;
				Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				Imgproc.medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				Imgproc.bilateralFilter(input, output, -1, radius, radius);
				break;
		}
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
	    Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	 * @param input The image on which to perform the Distance Transform.
	 * @param type The Transform.
	 * @param maskSize the size of the mask.
	 * @param output The image in which to store the output.
	 */
	private void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}

	public static List<OuterPortTarget> findTargets(ArrayList<MatOfPoint> targetRegions) {
		LinkedList<OuterPortTarget> targets = new LinkedList<OuterPortTarget>();

		OuterPortTarget opt = new OuterPortTarget(targetRegions);
		targets.add(opt);

		return targets;
	}

	public static void main(String[] args) {

		String[] filesToProcess = {
			"test_images/2020SampleVisionImages/BlueGoal-060in-Center.jpg                           ",
			"test_images/2020SampleVisionImages/BlueGoal-084in-Center.jpg                           ",
			"test_images/2020SampleVisionImages/BlueGoal-108in-Center.jpg                           ",
			"test_images/2020SampleVisionImages/BlueGoal-132in-Center.jpg                           ",
			"test_images/2020SampleVisionImages/BlueGoal-156in-Center.jpg                           ",
			"test_images/2020SampleVisionImages/BlueGoal-156in-Left.jpg                             ",
			"test_images/2020SampleVisionImages/BlueGoal-180in-Center.jpg                           ",
			"test_images/2020SampleVisionImages/BlueGoal-224in-Center.jpg                           ",
			"test_images/2020SampleVisionImages/BlueGoal-228in-ProtectedZone.jpg                    ",
			"test_images/2020SampleVisionImages/BlueGoal-330in-ProtectedZone.jpg                    ",
			"test_images/2020SampleVisionImages/BlueGoal-Far-ProtectedZone.jpg                      ",
			"test_images/2020SampleVisionImages/RedLoading-016in-Down.jpg                           ",
			"test_images/2020SampleVisionImages/RedLoading-030in-Down.jpg                           ",
			"test_images/2020SampleVisionImages/RedLoading-048in-Down.jpg                           ",
			"test_images/2020SampleVisionImages/RedLoading-048in.jpg                                 ",
			"test_images/2020SampleVisionImages/RedLoading-060in.jpg                                 ",
			"test_images/2020SampleVisionImages/RedLoading-084in.jpg                                 ",
			"test_images/2020SampleVisionImages/RedLoading-108in.jpg                                 ",
		};
	
		OuterPortTargetFromImage processor = new OuterPortTargetFromImage();
		for (String file : filesToProcess) {
			Mat img = Imgcodecs.imread(file);
			processor.process(img);

			for(OuterPortTarget opt : processor.getDetectedTargets()) {
				opt.drawOn(img);
			}

			HighGui.imshow(file, img);
		}
		HighGui.waitKey(10);
	}


}
