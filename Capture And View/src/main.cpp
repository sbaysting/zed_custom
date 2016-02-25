// Combined ZED Save Depth Sample with a custom viewer ran with PCL

// Author: Samuel Baysting
// Sample Code: Stereolabs

#define NOMINMAX

// Standard Libraries
#include <iomanip>
#include <signal.h>
#include <iostream>
#include <limits>
#include <thread>
#include <string>

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ZED Libraries
#include <zed/Camera.hpp>

// Boost Libraries
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

// PCL Libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;
namespace po = boost::program_options;


typedef struct SaveParamStruct {
	sl::POINT_CLOUD_FORMAT PC_Format;
	sl::DEPTH_FORMAT Depth_Format;
	std::string saveName;
	bool askSavePC;
	bool askSaveDepth;
	bool stop_signal;
} SaveParam;

sl::zed::Camera  * zed_ptr;
SaveParam *param;


std::string getFormatNamePC(sl::POINT_CLOUD_FORMAT f) {
	std::string str_;
	switch (f) {
	case sl::XYZ:
		str_ = "XYZ";
		break;
	case sl::PCD:
		str_ = "PCD";
		break;
	case sl::PLY:
		str_ = "PLY";
		break;
	case sl::VTK:
		str_ = "VTK";
		break;
	default:
		break;
	}
	return str_;
}

std::string getFormatNameD(sl::DEPTH_FORMAT f){
	std::string str_;
	switch (f)
	{
	case sl::PNG:
		str_ = "PNG";
		break;
	case sl::PFM:
		str_ = "PFM";
		break;
	case sl::PGM:
		str_ = "PGM";
		break;
	default:
		break;
	}
	return str_;
}

//UTILS fct to handle CTRL-C event
#ifdef _WIN32
BOOL CtrlHandler(DWORD fdwCtrlType) {
    switch (fdwCtrlType) {
            //Handle the CTRL-C signal.
        case CTRL_C_EVENT:
            printf("\nQuitting...\n");
            delete zed_ptr;
            exit(0);
        default:
            return FALSE;
    }
}
#else
void nix_exit_handler(int s) {
    printf("\nQuitting...\n");
    delete zed_ptr;
    exit(1);
}
#endif

void saveProcess(bool* quit)
{
	while (!param->stop_signal) {

		if (param->askSaveDepth){

			float max_value = std::numeric_limits<unsigned short int>::max();
			float scale_factor = max_value / zed_ptr->getDepthClampValue();

			std::cout << "Saving Depth Map " << param->saveName << " in " << getFormatNameD(param->Depth_Format) << " ..." << flush;
			sl::writeDepthAs(zed_ptr, param->Depth_Format, param->saveName, scale_factor);
			std::cout << "done" << endl;
			param->askSaveDepth = false;
		}
			
		if (param->askSavePC){
			std::cout << "Saving Point Cloud " << param->saveName << " in " << getFormatNamePC(param->PC_Format) << " ..." << flush;
			sl::writePointCloudAs(zed_ptr, param->PC_Format, param->saveName, true, false);
			std::cout << "done" << endl;
			param->askSavePC = false;
			*quit = true;
		}

		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}

//Main function
int main(int argc, char **argv) {

	string filename = "";
	string path = "./"; //Default output path
	string prefixPC = "PC_"; //Default output file prefix
	string prefixDepth = "Depth_"; //Default output file prefix
	string PCfile = ""; //Filename that the PCD file was saved under
	int resolution = 2; //Default resolution is set to HD720
	int quality = 2; //Default quality is set to QUALITY
	int device = -1; // Default CUDA device
	sl::zed::Camera* zed;
	int nbFrames = 0;
	sl::zed::MODE depth_mode = sl::zed::MODE::PERFORMANCE;

	//BOOST program options
	int opt;
	po::options_description desc("Available options");
	desc.add_options()
		("help", "Display available options")
		("filename,f", po::value< std::string >(), "SVO filename")
		("path,p", po::value< std::string >(), "Output path (can include output filename prefix)")
		("resolution,r", po::value<int>(&opt)->default_value(resolution), "ZED Camera resolution \narg: 0: HD2K   1: HD1080   2: HD720   3: VGA")
		("quality,q", po::value<int>(&opt)->default_value(quality), "Disparity Map quality \narg: 1: PERFORMANCE   2: QUALITY")
		("device,d", po::value<int>(&opt)->default_value(device), "CUDA device")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << "\n";
		return 1;
	}
	
	if (vm.count("filename")) {
		filename = vm["filename"].as<std::string>();
		cout << "Saving depth from " << filename << endl;
		zed = new sl::zed::Camera(filename);
		nbFrames = zed->getSVONumberOfFrames();
	}
	else {
		cout << "Saving depth from ZED camera" << endl;
		if (vm.count("resolution")) {
			resolution = vm["resolution"].as<int>();
			switch (resolution) {
			case 0: cout << "Resolution set to HD2K" << endl;
				break;
			case 1: cout << "Resolution set to HD1080" << endl;
				break;
			case 2: cout << "Resolution set to HD720" << endl;
				break;
			case 3: cout << "Resolution set to VGA" << endl;
				break;
			}
		}
		zed = new sl::zed::Camera(static_cast<sl::zed::ZEDResolution_mode> (resolution));
	}
	if (vm.count("path")) {
		string p = vm["path"].as<std::string>();
		path = (p.back() == '/' || p.back() == '\\') ? p : p + '\\';
	}
	if (vm.count("quality")) {
		quality = vm["quality"].as<int>();
		switch (quality) {
		case 1:
			cout << "Quality set to PERFORMANCE" << endl;
			depth_mode = sl::zed::MODE::PERFORMANCE;
			break;
		case 2:
			cout << "Quality set to QUALITY" << endl;
			depth_mode = sl::zed::MODE::QUALITY;
			break;
		default:
			cout << "Invalid depth quality" << endl;
			break;
		}
	}
	if (vm.count("device")) {
		device = vm["device"].as<int>();
	}

	sl::zed::ERRCODE err = zed->init(depth_mode, device, true);

	zed_ptr = zed;

	//ERRCODE display
	cout << errcode2str(err) << endl;

	//Quit if an error occurred
	if (err != sl::zed::ERRCODE::SUCCESS) {
		delete zed;
		return 1;
	}

	//CTRL-C (= kill signal) handler
#ifdef _WIN32
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE);
#else // unix
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = nix_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
#endif

    char key = ' '; // key pressed
	int width = zed_ptr->getImageSize().width;
	int height = zed_ptr->getImageSize().height;
    cv::Size size(width, height); // image size
    cv::Mat depthDisplay(size, CV_8UC1); // normalized depth to display
	
	bool printHelp = false;
	std::string helpString = "[d] save Depth, [P] Save Point Cloud, [m] change format PC, [n] change format Depth, [q] quit";

	int depth_clamp = 5000;
	zed_ptr->setDepthClampValue(depth_clamp);
	
	int mode_PC = 1;
	int mode_Depth = 0;
	
	param = new SaveParam();
	param->askSavePC = false;
	param->askSaveDepth = false;
	param->stop_signal = false;
	param->PC_Format = static_cast<sl::POINT_CLOUD_FORMAT>(mode_PC);
	param->Depth_Format = static_cast<sl::DEPTH_FORMAT>(mode_Depth);

	bool quit_ = false;

	boost::thread grab_thread(saveProcess, &quit_);	
	
	std::cout<<" Press 'p' to save Point Cloud in PCD format and display"<<std::endl;

	int count = 0;
	while (!quit_ && (zed_ptr->getSVOPosition() <= nbFrames)) {

		zed_ptr->grab(sl::zed::SENSING_MODE::RAW, 1, 1);
		
		slMat2cvMat(zed_ptr->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(depthDisplay);

		if (printHelp) // write help text on the image if needed
			cv::putText(depthDisplay, helpString, cv::Point(20, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(111, 111, 111, 255), 2);

        cv::imshow("Depth", depthDisplay);
        key = cv::waitKey(5);

		switch (key)
		{
		case 'p':
		case 'P':
			PCfile = prefixPC + to_string(count);
			param->saveName = path + PCfile;
			PCfile = PCfile + ".pcd";
			param->askSavePC = true;
		break;

		case 'd':
		case 'D':
			param->saveName = path + prefixDepth + to_string(count);
			param->askSaveDepth = true;
		break;

		case 'm':
		case 'M':
		{
			mode_PC++;
			param->PC_Format = static_cast<sl::POINT_CLOUD_FORMAT>(mode_PC % 4);
			std::cout << "Format Point Coud " << getFormatNamePC(param->PC_Format) << std::endl;
		}
		break;

		case 'n':
		case 'N':
		{
			mode_Depth++;
			param->Depth_Format = static_cast<sl::DEPTH_FORMAT>(mode_Depth % 3);
			std::cout << "Format Depth " << getFormatNameD(param->Depth_Format) << std::endl;
		}
		break;

		case 'h': // print help
		case 'H':
			printHelp = !printHelp;
			cout << helpString << endl;
			break;

		case 'q': // QUIT
		case 'Q':
		case 27:
			quit_ = true;
			break;			
		}
		count++;
    }

	param->stop_signal = true;
	grab_thread.join();
	
	delete zed_ptr;

	// Define a point cloud pointer
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Load PCD point cloud file
	std::cout << "Opening point cloud: " << PCfile << std::endl;
    pcl::io::loadPCDFile (PCfile, *cloud);
	// Define a cloud viewer
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	// Show point cloud
	viewer.showCloud(cloud);

	// Loop to do things while the point cloud viewer is still on
	while (!viewer.wasStopped())
	{
	}

    return 0;
}
