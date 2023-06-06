#include <bagproc/bag2video.h>

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

std::string replaceSlash(std::string str){
    for(int i=0; i<str.length(); i++){
        if(str[i] == '/'){
            str[i] = '_';
        }
    }
    return str;
}

std::string removeStrigula(const std::string& inputStr) {
    std::string str(inputStr);
    str.erase(std::remove(str.begin(), str.end(), '-'), str.end());
    return str;
}

int main(int argc, char **argv)
{
    std::string bagPath;

    if(argc != 2)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --bagPath");
        std::cout << "Enter the bagfile complete path:" << std::endl;
        std::cin >> bagPath;
    }
    else
    {
        bagPath.assign(argv[1]);
    }

    ROS_INFO("[bag2video] --bagPath: %s", bagPath.c_str());

    if(boost::filesystem::exists(bagPath) == false)  // Check whether the bagfile exists.
    {
        ROS_ERROR("Specified rosbag file not exist.-- %s \n", bagPath.c_str());
        return -1;
    }
    std::string filePath = boost::filesystem::path(bagPath).parent_path().string();
    std::string filename = boost::filesystem::path(bagPath).stem().string();
    filename = removeStrigula(filename);

    ros::init(argc, argv, "bag2video");

    std::string outPath = filePath + "/output_videos";
    if (!check_targetDir(outPath))
    {
        ROS_ERROR("check output_video DIR error, outPath: %s", outPath.c_str());
        return -1;
    }

    rosbag::Bag bag;
    bag.open(bagPath);

    setbuf(stdout, NULL);     // Set to no buffering
    std::cout << "\033[?25l"; // Hidden cursor

    uint16_t frames_cpr = 0, frames_img = 0;
    std::string videoName1 = outPath + filename + "_compressed" + ".mp4";
    std::string videoName2 = outPath + filename + "_raw" + ".mp4";
    cv::VideoWriter vWriter1, vWriter2;
    vWriter1.open(videoName1, cv::VideoWriter::fourcc('h', '2', '6', '4'), 30, cv::Size(640, 480));
    vWriter2.open(videoName2, cv::VideoWriter::fourcc('h', '2', '6', '4'), 30, cv::Size(640, 480));

    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        sensor_msgs::CompressedImageConstPtr c_img_ptr = m.instantiate<sensor_msgs::CompressedImage>();
        if (c_img_ptr != nullptr)
        {
            std::cout << "\r[bag2video] -Compressed Start:---->>  " << ++frames_cpr;
            cv::Mat img = cv_bridge::toCvCopy(c_img_ptr, sensor_msgs::image_encodings::BGR8)->image;
            vWriter1 << img;
        }

        sensor_msgs::ImageConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
        if (img_ptr != nullptr)
        {
            std::cout << "\r[bag2video] Start:-->>  " << ++frames_img;
            cv::Mat img = cv_bridge::toCvShare(img_ptr, sensor_msgs::image_encodings::BGR8)->image;
            vWriter2 << img;
        }
    }
    vWriter1.release();
    vWriter2.release();
    bag.close();
    std::cout << "\nbag2video complished extract: [--compressed images " << frames_cpr << "], [--raw images " << frames_img << "]." << std::endl
              << "\033[?25h"; // Show cursor.
    if(frames_cpr < 10) // Not enough frames to synthesize video
    {
        boost::filesystem::remove(videoName1);
        ROS_INFO("Topic 'image_compressed' not in rosbag file.");
    }
    else ROS_INFO("rosbag convert to compressed video successed.\nOutput path: %s", videoName1.c_str());

    if(frames_img < 10) // Not enough frames to synthesize video
    {
        boost::filesystem::remove(videoName2);
        ROS_INFO("Topic 'image_raw' not in rosbag file.");
    }
    else ROS_INFO("rosbag convert to compressed video successed.\nOutput path: %s", videoName2.c_str());

    return 0;
}