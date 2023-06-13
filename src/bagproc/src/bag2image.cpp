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

// Check whether the target folder exists, and create it if it does not
bool check_targetDir(std::string& path)
{
    bool success = true;
    if(!checkPath_OK(path))
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
    std::string bagPath;
    uint8_t interval;

    if(argc != 3)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --bagPath --interval(Image extraction frame interval)");
        std::cout << "Enter the bagfile complete path:" << std::endl;
        std::cin >> bagPath;
        std::cout << "Enter the file image extraction frame interval:" << std::endl;
        std::cin >> interval;
    }
    else
    {
        bagPath.assign(argv[1]);
        interval = atoi(argv[2]);
    }
    std::cout << "[bag2video INFO]:" << " --bagPath:" << bagPath << " --interval(frames):" << unsigned(interval) << std::endl;

    ros::init(argc, argv, "bag2video");
    if(boost::filesystem::exists(bagPath) == false)  // Check whether the bagfile exists.
    {
        ROS_ERROR("Specified rosbag file not exist.-- %s \n", bagPath.c_str());
        return -1;
    }

    std::string filePath = boost::filesystem::path(bagPath).parent_path().string();
    std::string filename = boost::filesystem::path(bagPath).stem().string();
    filename = removeUnderscores(filename);

    std::string outPath = filePath + "/output_images";
    if(!check_targetDir(outPath))
    {
        ROS_ERROR("Check output_image DIR error, outPath: %s", outPath.c_str());
        return -1;
    }

    rosbag::Bag bag;
    bag.open(bagPath);

    std::string imgPath;
    setbuf(stdout, NULL);   // Set to no buffering
    std::cout << "\033[?25l";   // Hidden cursor
    imgPath = outPath + filename;
    if (!check_image_dir(imgPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", imgPath.c_str());
        return -1;
    }
    uint8_t count_cpr = 0, count_img = 0;
    uint16_t frames_cpr = 0, frames_img = 0;

    // if (std::string imgTopic.find("compressed") != std::string::npos) 
    // find specified content in string
    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        std::cout << "\r[bag2image] -Start:---->>  -image_compressed:" << frames_cpr << " -image_raw:" << frames_img;
        
        sensor_msgs::CompressedImageConstPtr c_img_ptr = m.instantiate<sensor_msgs::CompressedImage>();
        if (c_img_ptr != nullptr)
        {
            if ((++count_cpr) % interval == 0)
            {
                cv::Mat img = cv_bridge::toCvCopy(c_img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                std::stringstream ss;
                ss << imgPath << "cpr_" << ++frames_cpr << ".png";
                cv::imwrite(ss.str(), img);
            }
        }

        sensor_msgs::ImageConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
        if (img_ptr != nullptr)
        {
            if ((++count_img) % interval == 0)
            {
                cv::Mat img = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::RGB8)->image;
                cv::Mat image;
                cv::cvtColor(img, image, CV_RGB2BGR);
                std::stringstream ss;
                ss << imgPath << "img_" << ++frames_img << ".png";
                cv::imwrite(ss.str(), image);
            }
        }
    }
    bag.close();
    std::cout << "\nbag2image complished extract: [--compressed images " << frames_cpr << "], [--raw images " << frames_img << "]." << std::endl
              << "\033[?25h"; // Show cursor.
    ROS_INFO("rosbag convert to image successed.\nOutput path: %s", imgPath.c_str());

    return 0;
}