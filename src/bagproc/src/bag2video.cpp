#include <bagproc/bag2video.h>

std::string bagname = "Test_compressed.bag";

bool checkPath_OK(std::string &path)
{
    if(path.back() != '/') {    // Is the end of the string a slash?
        path.append("/");
    }
    DIR *dir = opendir(path.c_str());
    if(dir == NULL)
    {
        ROS_WARN("A nonexistent path, check it. %s", path.c_str());
        return false;
    }
    return true;
}

bool clear_dir(const std::string& path) {
    DIR* dir = opendir(path.c_str());
    if (dir == nullptr) {
        return false;
    }
    bool success = true;
    while (dirent* entry = readdir(dir)) {
        if (entry->d_type != DT_DIR && entry->d_type != DT_REG) {
            continue;
        }
        std::string name = entry->d_name;
        if (name == "." || name == "..") {
            continue;
        }
        std::string entry_path = path + "/" + name;
        if (entry->d_type == DT_DIR) {
            success = clear_dir(entry_path) && success;
        } else {
            success = (unlink(entry_path.c_str()) == 0) && success;
        }
    }
    closedir(dir);
    return success;
}

bool create_dir(const std::string& path) {
    return mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0;
}

bool check_dir(std::string& path)
{
    bool success = true;
    if(checkPath_OK(path))
    {
        success = clear_dir(path);
    }
    else
    {
        success = create_dir(path);
    }
    return success;
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
        ROS_WARN_STREAM("Error grguiments. Todo: --imgTopic --filePath");
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

        time_t now = time(NULL);
        struct tm *local_tm = localtime(&now);
        std::stringstream time_name;
        time_name << local_tm->tm_year + 1900 << "-"
                    << std::setw(2) << std::setfill('0') << local_tm->tm_mon + 1 << "-"
                    << std::setw(2) << std::setfill('0') << local_tm->tm_mday << " "
                    << std::setw(2) << std::setfill('0') << local_tm->tm_hour << ":"
                    << std::setw(2) << std::setfill('0') << local_tm->tm_min << ":"
                    << std::setw(2) << std::setfill('0') << local_tm->tm_sec;

        std::string videoPath = filePath + "output_videos";
        if(!check_dir(videoPath))
        {
            ROS_ERROR("check video DIR error, videoPath: %s", videoPath.c_str());
            return -1;
        }

        setbuf(stdout, NULL);   // Set to no buffering
        std::cout << "\033[?25l";   // Hidden cursor
        uint16_t frameCount = 0;
        cv::VideoWriter videoWriter;
        std::string videoName;

        if(imgTopic.find("compressed") != std::string::npos)
        {
            videoName = videoPath + "compressed_" + time_name.str() + ".mp4";
            videoWriter.open(videoName, cv::VideoWriter::fourcc('h','2','6','4'), 30, cv::Size(640,480));
            for (rosbag::MessageInstance const m : rosbag::View(bag_))
            {
                sensor_msgs::CompressedImageConstPtr c_img_ptr = m.instantiate<sensor_msgs::CompressedImage>();
                if (c_img_ptr != nullptr)
                {
                    std::cout << "\r rosbag2video Start:-->>  " << frameCount++;
                    cv::Mat img = cv_bridge::toCvCopy(c_img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                    videoWriter << img;
                }
            }
        }
        else
        {
            videoName = videoPath + "nomal_" + time_name.str() + ".mp4";
            videoWriter.open(videoName, cv::VideoWriter::fourcc('h','2','6','4'), 30, cv::Size(640,480));
            for (rosbag::MessageInstance const m : rosbag::View(bag_))
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
        ROS_INFO("rosbag to video successed. Output path: %s",videoName.c_str());
    }

    return 0;
}