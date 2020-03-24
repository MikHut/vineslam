// An example to classify image.
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "../detection/engine.h"
#include "label_utils.h"
#include "model_utils.h"
#include "../test_utils.h"

ABSL_FLAG(std::string, model_path,
          "/home/andre-criis/Source/coral/detect/train/train_res/mv1_25-10/"
          "output_tflite_graph.tflite",
          "Path to the tflite model.");

ABSL_FLAG(std::string, image_path,
          "/home/andre-criis/Source/coral/detect/train/images_eval/"
          "rotated_left0679.bmp",
          "Path to the image to be classified.");

ABSL_FLAG(std::string, labels_path,
          "/home/andre-criis/Source/coral/detect/train/train_res/mv1_25-10/"
          "labels.txt",
          "Path to the imagenet labels.");

void ObjectDetection(const std::string& model_path,
                     const std::string& image_path,
                     const std::string& labels_path)
{
	// Load the model.
	coral::DetectionEngine engine(model_path);

	std::vector<int> input_tensor_shape = engine.get_input_tensor_shape();
	/* // Read the image. */
	std::vector<uint8_t> input_tensor = coral::GetInputFromImage(
	    image_path,
	    {input_tensor_shape[1], input_tensor_shape[2], input_tensor_shape[3]});

	// Read the label file.
	auto labels = coral::ReadLabelFile(labels_path);

	auto results = engine.DetectWithInputTensor(input_tensor);
	for (auto result : results) {
	  std::cout << "---------------------------" << std::endl;
	  std::cout << labels[result.label] << std::endl;
    std::cout << result.DebugString() << std::endl;
	  std::cout << "Score: " << result.score << std::endl;
	}
}

int main(int argc, char* argv[])
{
	ObjectDetection(absl::GetFlag(FLAGS_model_path),
	                absl::GetFlag(FLAGS_image_path),
	                absl::GetFlag(FLAGS_labels_path));
}
