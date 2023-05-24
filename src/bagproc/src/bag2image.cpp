#include <bagproc/bag2image.h>

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

bool check_image_dir(std::string& path)
{
    bool success = true;
    if(checkPath_OK(path))
    {
        success = clear_dir(path);
        ROS_INFO("clear_dir :%s %d",path.c_str(), success);
    }
    else
    {
        success = create_dir(path);
        ROS_INFO("create_dir :%s %d",path.c_str(), success);
    }
    return success;
}

int main(int argc, char **argv)
{
    std::string imgTopic, filePath;
    uint8_t interval;

    if(argc != 4)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --imgTopic --filePath --interval(Image extraction frame interval)");
        std::cout << "Enter the image topic:\n" << std::endl;
        std::cin >> imgTopic;
        std::cout << "Enter the file complete path:\n" << std::endl;
        std::cin >> filePath;
        std::cout << "Enter the file image extraction frame interval:\n" << std::endl;
        std::cin >> interval;
    }
    else
    {
        imgTopic.assign(argv[1]);
        filePath.assign(argv[2]);
        interval = atoi(argv[3]);
    }
    ROS_INFO("[rosbag2video] --imgTopic: %s  --filePath: %s  --interval: %d", imgTopic.c_str(), filePath.c_str(), interval);

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
        std::string outPath = filePath + "output_images";
        if(!check_image_dir(outPath))
        {
            ROS_ERROR("check image DIR error, imgPath: %s", outPath.c_str());
            return -1;
        }

        uint16_t frameCount = 0, countTemp = 0;
        std::stringstream ss;
        setbuf(stdout, NULL);   // Set to no buffering
        std::cout << "\033[?25l";   // Hidden cursor

        time_t now = time(NULL);
        struct tm *local_tm = localtime(&now);
        std::stringstream time_name;
        time_name << local_tm->tm_year + 1900 << "-"
                    << std::setw(2) << std::setfill('0') << local_tm->tm_mon + 1 << "-"
                    << std::setw(2) << std::setfill('0') << local_tm->tm_mday << " "
                    << std::setw(2) << std::setfill('0') << local_tm->tm_hour << ":"
                    << std::setw(2) << std::setfill('0') << local_tm->tm_min << ":"
                    << std::setw(2) << std::setfill('0') << local_tm->tm_sec;
        std::string imgPath;

        if(imgTopic.find("compressed") != std::string::npos)
        {
            imgPath = outPath + "compressed_frames_" + time_name.str();
            if(!check_image_dir(imgPath))
            {
                ROS_ERROR("check image DIR error, imgPath: %s", imgPath.c_str());
                return -1;
            }
            
            for (rosbag::MessageInstance const m : rosbag::View(bag_))
            {
                sensor_msgs::CompressedImageConstPtr c_img_ptr = m.instantiate<sensor_msgs::CompressedImage>();
                if (c_img_ptr != nullptr)
                {
                    if((++countTemp)%interval == 0)
                    {
                        std::cout << "\r rosbag2image Compressed images Start:-->>  " << frameCount++;
                        cv::Mat img = cv_bridge::toCvCopy(c_img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                        ss << imgPath << c_img_ptr->header.stamp.toNSec() << ".png";
                        cv::imwrite(ss.str(), img);
                        ss.clear();
                        ss.str("");
                    }
                }
            }
        }
        else
        {
            imgPath = outPath + "frames_" + time_name.str();
            if(!check_image_dir(imgPath))
            {
                ROS_ERROR("check image DIR error, imgPath: %s", imgPath.c_str());
                return -1;
            }
            for (rosbag::MessageInstance const m : rosbag::View(bag_))
            {
                sensor_msgs::ImageConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
                if (img_ptr != nullptr)
                {
                    if ((++countTemp) % interval == 0)
                    {
                        std::cout << "\r rosbag2image Images Start:-->>  " << frameCount++;
                        cv::Mat img = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                        ss << imgPath << img_ptr->header.stamp.toNSec() << ".png";
                        cv::imwrite(ss.str(), img);
                        ss.clear();
                        ss.str("");
                    }
                }
            }
        }
        ROS_INFO("\nrosbag to video successed. Output path: %s",imgPath.c_str());
        bag_.close();
        std::cout << "\n rosbag2image Complished." << std::endl << "\033[?25h"; // Show cursor.
        
    }

    return 0;
}