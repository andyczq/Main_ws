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

    ROS_INFO("[rs_bag2video] --bagPath: %s", bagPath.c_str());

    if(boost::filesystem::exists(bagPath) == false)  // Check whether the bagfile exists.
    {
        ROS_ERROR("Specified rosbag file not exist.-- %s \n", bagPath.c_str());
        return -1;
    }
    ros::init(argc, argv, "rs_bag2video");

    std::string filePath = boost::filesystem::path(bagPath).parent_path().string();
    std::string filename = boost::filesystem::path(bagPath).stem().string();
    filename = removeStrigula(filename);

    std::string outPath = filePath + "/output_videos";
    if (!check_targetDir(outPath))
    {
        ROS_ERROR("check output_video DIR error, outPath: %s", outPath.c_str());
        return -1;
    }

    std::string videoPath = outPath + filename;
    if (!check_targetDir(videoPath))
    {
        ROS_ERROR("check video DIR error, videoPath: %s", videoPath.c_str());
        return -1;
    }

    rosbag::Bag bag;
    bag.open(bagPath);

    setbuf(stdout, NULL);     // Set to no buffering
    std::cout << "\033[?25l"; // Hidden cursor

    std::string color_video = videoPath + "color_" + filename + ".mp4";
    std::string depth_video = outPath + "depth_" + filename + ".mp4";
    std::string align_video = outPath + "align_" + filename + ".mp4";
    cv::VideoWriter color_writer, depth_writer, align_writer;
    color_writer.open(color_video, cv::VideoWriter::fourcc('h', '2', '6', '4'), 30, cv::Size(640, 480));
    depth_writer.open(depth_video, cv::VideoWriter::fourcc('h', '2', '6', '4'), 30, cv::Size(640, 480));
    align_writer.open(align_video, cv::VideoWriter::fourcc('h', '2', '6', '4'), 30, cv::Size(640, 480));
    uint16_t color_count = 0, depth_count = 0, align_count = 0;

    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        sensor_msgs::CompressedImageConstPtr image_ptr = m.instantiate<sensor_msgs::CompressedImage>();
        if (image_ptr != nullptr)
        {
            std::cout << "\r[rs_bag2video] -Start:---->>  -color:" << color_count << " -depth:" << depth_count << " -align:" << align_count;

            try
            {
                cv::Mat image = cv::imdecode(cv::Mat(image_ptr->data), cv::IMREAD_COLOR);

                if(m.getTopic() == "/camera/color/image_raw/compressed")
                {
                    color_count++;
                    color_writer << image;
                }
                else if(m.getTopic() == "/camera/depth/image_rect_raw/compressed")
                {
                    depth_count++;
                    depth_writer << image;
                }
                else if(m.getTopic() == "/camera/aligned_depth_to_color/image_raw/compressed")
                {
                    align_count++;
                    align_writer << image;
                }
            }
            catch(const cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
            
        }

    }
    color_writer.release();
    depth_writer.release();
    align_writer.release();
    bag.close();
    std::cout << "\n[rs_bag2video] -Ended with:---->>  -color:" << color_count << " -depth:" << depth_count << " -align:" << align_count << "\033[?25h"; // Show cursor.
    if(color_count < 10) // Not enough frames to synthesize video
    {
        boost::filesystem::remove(color_video);
        ROS_INFO("Topic 'color_image' not in rosbag file.");
    }
    else ROS_INFO("rosbag convert to color_video successed.\nOutput path: %s", color_video.c_str());

    if(depth_count < 10) // Not enough frames to synthesize video
    {
        boost::filesystem::remove(depth_video);
        ROS_INFO("Topic 'depth_image' not in rosbag file.");
    }
    else ROS_INFO("rosbag convert to depth_video successed.\nOutput path: %s", depth_video.c_str());

    if(align_count < 10) // Not enough frames to synthesize video
    {
        boost::filesystem::remove(align_video);
        ROS_INFO("Topic 'depth_image' not in rosbag file.");
    }
    else ROS_INFO("rosbag convert to align_video successed.\nOutput path: %s", align_video.c_str());

    return 0;
}