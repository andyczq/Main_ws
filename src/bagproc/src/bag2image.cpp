#include <bagproc/bag2image.h>

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
        if(success) {
            ROS_INFO("clear_dir :%s success.",path.c_str());
        }
        else {
            ROS_WARN("clear_dir :%s failure.",path.c_str());
        }
    }
    else
    {
        success = create_dir(path);
        if(success) {
            ROS_INFO("create_dir :%s success.",path.c_str());
        }
        else {
            ROS_WARN("create_dir :%s failure.",path.c_str());
        }
    }
    return success;
}

std::string removeUnderscores(const std::string& inputStr) {
    std::string str(inputStr);
    str.erase(std::remove(str.begin(), str.end(), '-'), str.end());
    return str;
}

int main(int argc, char **argv)
{
    std::string imgTopic, bagPath;
    uint8_t interval;

    if(argc != 4)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --imgTopic --filePath --interval(Image extraction frame interval)");
        std::cout << "Enter the image topic:\n" << std::endl;
        std::cin >> imgTopic;
        std::cout << "Enter the bagfile complete path:\n" << std::endl;
        std::cin >> bagPath;
        std::cout << "Enter the file image extraction frame interval:\n" << std::endl;
        std::cin >> interval;
    }
    else
    {
        imgTopic.assign(argv[1]);
        bagPath.assign(argv[2]);
        interval = atoi(argv[3]);
    }
    std::cout << "[rosbag2video] --imgTopic:" << imgTopic << " --bagPath:" << bagPath << " --interval(frames):" << unsigned(interval) << std::endl;

    ros::init(argc, argv, "rosbag2video");
    if(boost::filesystem::exists(bagPath) == false)  // Check whether the bagfile exists.
    {
        ROS_ERROR("The rosbag file not exist.-- %s \n", bagPath.c_str());
        return -1;
    }

    std::string filePath = boost::filesystem::path(bagPath).parent_path().string();
    std::string filename = boost::filesystem::path(bagPath).stem().string();
    filename = removeUnderscores(filename);

    std::string outPath = filePath + "/output_images";
    if(!check_image_dir(outPath))
    {
        ROS_ERROR("check output_image DIR error, outPath: %s", outPath.c_str());
        return -1;
    }

    rosbag::Bag bag;
    bag.open(bagPath);

    std::string imgPath;
    uint16_t frameCount = 0;
    setbuf(stdout, NULL);   // Set to no buffering
    std::cout << "\033[?25l";   // Hidden cursor
    imgPath = outPath + filename;
    if (!check_image_dir(imgPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", imgPath.c_str());
        return -1;
    }
    uint8_t countTemp = 0;

    if (imgTopic.find("compressed") != std::string::npos)
    {
        for (rosbag::MessageInstance const m : rosbag::View(bag))
        {
            sensor_msgs::CompressedImageConstPtr c_img_ptr = m.instantiate<sensor_msgs::CompressedImage>();
            if (c_img_ptr != nullptr)
            {
                if ((++countTemp) % interval == 0)
                {
                    std::cout << "\r rosbag2image Compressed images Start:-->>  " << ++frameCount;
                    cv::Mat img = cv_bridge::toCvCopy(c_img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                    std::stringstream ss;
                    ss << imgPath << "cpr_" << frameCount << ".png";
                    cv::imwrite(ss.str(), img);
                }
            }
        }
    }
    else
    {
        for (rosbag::MessageInstance const m : rosbag::View(bag))
        {
            sensor_msgs::ImageConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
            if (img_ptr != nullptr)
            {
                if ((++countTemp) % interval == 0)
                {
                    std::cout << "\r rosbag2image Images Start:-->>  " << ++frameCount;
                    cv::Mat img = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                    std::stringstream ss;
                    ss << imgPath << "img_" << frameCount << ".png";
                    cv::imwrite(ss.str(), img);
                }
            }
        }
    }
    bag.close();
    ROS_INFO("\nrosbag convert to image successed.\nOutput path: %s", imgPath.c_str());
    std::cout << "\nrosbag2image complished extract " << frameCount << " frames" << std::endl
              << "\033[?25h"; // Show cursor.

    return 0;
}