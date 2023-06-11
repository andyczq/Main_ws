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

// Check whether the target folder exists, and clear it if it does not
bool check_imageDir(std::string& path)
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
    std::string bagPath, filter;
    uint8_t interval;

    if(argc < 3)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --bagPath --interval(Image extraction frame interval)");
        std::cout << "Enter the bagfile complete path:" << std::endl;
        std::cin >> bagPath;
        std::cout << "Enter the file image extraction frame interval:" << std::endl;
        std::cin >> interval;
    }
    else if(argc == 3)
    {
        bagPath.assign(argv[1]);
        interval = atoi(argv[2]);
    }
    else
    {
        bagPath.assign(argv[1]);
        interval = atoi(argv[2]);
        filter.assign(argv[3]);
    }

    bool colorizer =false;
    if(filter == "colorizer")
        colorizer = true;
    std::cout << "[rs_bag2image] INFO:" << "\n --bagPath:" << bagPath << "\n --interval(frames):" << unsigned(interval) << " --colorizer:" << std::boolalpha << colorizer << std::endl;

    ros::init(argc, argv, "rs_bag2image");
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
    imgPath = outPath + filename;
    if (!check_targetDir(imgPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", imgPath.c_str());
        return -1;
    }

    std::string colorPath = imgPath +  "color";
    if (!check_imageDir(colorPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", colorPath.c_str());
        return -1;
    }

    std::string depthPath = imgPath +  "depth";
    if (!check_imageDir(depthPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", depthPath.c_str());
        return -1;
    }

    std::string alignPath = imgPath +  "align";
    if (!check_imageDir(alignPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", alignPath.c_str());
        return -1;
    }

    uint8_t color_cap = 0, depth_cap = 0, align_cap = 0;;
    uint16_t color_count = 0, depth_count = 0, align_count = 0;

    setbuf(stdout, NULL);   // Set to no buffering
    std::cout << "\033[?25l\n";   // Hidden cursor
    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        sensor_msgs::CompressedImageConstPtr image_ptr = m.instantiate<sensor_msgs::CompressedImage>();
        if (image_ptr != nullptr)
        {
            std::cout << "\r[rs_bag2image] -Start:---->>  -color:" << color_count << " -depth:" << depth_count << " -align:" << align_count;
            try
            {
                cv::Mat image = cv::imdecode(cv::Mat(image_ptr->data), cv::IMREAD_COLOR);   // IMREAD_ANYDEPTH
                if(m.getTopic() == "/camera/color/image_raw")
                {
                    if ((++color_cap) % interval == 0)
                    {
                        std::stringstream imgName;
                        imgName << colorPath << "color_"  << std::setw(5) << std::setfill('0') << ++color_count << ".png";
                        cv::imwrite(imgName.str(), image);
                    }
                }
                else if(m.getTopic() == "/camera/depth/image_rect_raw")
                {
                    if ((++depth_cap) % interval == 0)
                    {
                        std::stringstream imgName;
                        imgName << depthPath << "depth_"  << std::setw(5) << std::setfill('0') << ++depth_count << ".png";
                        // if(!colorizer)
                        // {
                        //     image = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                        //     //  normalized the depth image to between 0 and 255
                        //     // double min_value, max_value;
                        //     // cv::minMaxLoc(image, &min_value, &max_value);
                        //     // cv::Mat temp_image;
                        //     // cv::convertScaleAbs(image, temp_image, 255.0 / max_value);
                        //     // cv::imwrite(imgName.str(), temp_image);
                        // }
                        cv::imwrite(imgName.str(), image);
                    }
                }
                else if(m.getTopic() == "/camera/aligned_depth_to_color/image_raw")
                {
                    if ((++align_cap) % interval == 0)
                    {
                        std::stringstream imgName;
                        imgName << alignPath << "align_"  << std::setw(5) << std::setfill('0') << ++align_count << ".png";
                        // if(!colorizer)
                        //     image = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                        cv::imwrite(imgName.str(), image);
                    }
                }
            }
            catch(const cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }
    }
    bag.close();
    std::cout << "\nbag2image complished extract: [--color_images " << color_count << "], [--depth_images " << depth_count << "], [--align_images " << align_count << "]." << std::endl
              << "\033[?25h"; // Show cursor.
    ROS_INFO("rosbag convert to image successed.\nOutput path: %s", imgPath.c_str());

    return 0;
}