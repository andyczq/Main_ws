#include <bag2video.h>

std::string bagname = "Test_compressed.bag";

bool checkPath_OK(std::string &path)
{
    if(path.back() != '/') {    // Is the end of the string a slash?
        path.append("/");
    }
    DIR *dir = opendir(path.c_str());
    if(dir == NULL)
    {
        ROS_ERROR("A nonexistent path, check it. %s", path.c_str());
        return false;
    }
    return true;
}

std::string replaceSlash(std::string str){
    for(int i=0; i<str.length(); i++){
        if(str[i] == '/'){
            str[i] = '_';
        }
    }
    return str;
}

int main(int argc, char **argv)
{
    std::string imgTopic, filePath;

    if(argc != 3)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --imgTopic --filePath --format(default:0 Compressed)");
        std::cout << "Enter the image topic:\n" << std::endl;
        std::cin >> imgTopic;
        std::cout << "Enter the file complete path:\n" << std::endl;
        std::cin >> filePath;
    }
    else
    {
        imgTopic.assign(argv[1]);
        filePath.assign(argv[2]);
    }

    ROS_INFO("[rosbag2video] --imgTopic: %s  --filePath: %s", imgTopic.c_str(), filePath.c_str());
    
    if(checkPath_OK(filePath))
    {
        ros::init(argc, argv, "rosbag2video");
        std::string bagPath = filePath + bagname;
        rosbag::Bag bag_;
        try {
            bag_.open(bagPath);
        }
        catch(cv_bridge::Exception e) {
            ROS_ERROR("%s \n", e.what());
            ROS_WARN("Could not open the rosbag file:%s \n", bagPath.c_str());
            return -1;
        }

        std::string videoPath = filePath + "output" + replaceSlash(imgTopic) + ".mp4";
        cv::VideoWriter videoWriter;
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('h','2','6','4'), 30, cv::Size(640,480));
        setbuf(stdout, NULL);   // Set to no buffering
        std::cout << "\033[?25l";   // Hidden cursor
        uint16_t frameCount = 0;

        for (rosbag::MessageInstance const m : rosbag::View(bag_))
        {
            if(imgTopic.find("compressed") != std::string::npos)
            {
                sensor_msgs::CompressedImageConstPtr c_img_ptr = m.instantiate<sensor_msgs::CompressedImage>();
                if (c_img_ptr != nullptr)
                {
                    std::cout << "\r rosbag2video Start:-->>  " << frameCount++;
                    cv::Mat img = cv_bridge::toCvCopy(c_img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                    videoWriter << img;
                }
            }
            else
            {
                sensor_msgs::ImageConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
                if (img_ptr != nullptr)
                {
                    std::cout << "\r rosbag2video Start:-->>  " << frameCount++;
                    cv::Mat img = cv_bridge::toCvShare(img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                    videoWriter << img;
                }
            }
        }
        videoWriter.release();
        bag_.close();
        std::cout << "\n rosbag2video Complished." << std::endl << "\033[?25h"; // Show cursor.
        ROS_INFO("rosbag to video successed. Output path: %s",videoPath.c_str());
    }

    return 0;
}