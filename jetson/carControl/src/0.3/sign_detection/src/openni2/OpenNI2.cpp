#include "openni2/OpenNI2.h"
using namespace framework;
OpenNI2::OpenNI2()
{

}

OpenNI2::~OpenNI2()
{

}

bool OpenNI2::init(bool is_colorize_disp, bool is_fixed_max_disp, int image_mode, bool *retrived_img_flags) {
    this->is_colorize_disp = is_colorize_disp;
    this-> is_fixed_max_disp = is_fixed_max_disp;
    this-> image_mode = image_mode;
    std::cout << "Openning Device ..." << std::endl;
    capture.open(cv::CAP_OPENNI2);
    if ( !capture.isOpened()) {
        std::cout << "Can not open a capture object" << std::endl;
        return false;
    }
    
    if (image_mode >= 0) {
        bool mode_res = false;
        switch (image_mode) {
            case 0:
                mode_res = capture.set(cv::CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv::CAP_OPENNI_VGA_30HZ);
                break;
            case 1:
                mode_res = capture.set(cv::CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv::CAP_OPENNI_SXGA_15HZ);
                break;
            case 2:
                mode_res = capture.set(cv::CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv::CAP_OPENNI_SXGA_30HZ );
                break;
                //The following modes are only supported by the Xtion Pro Live
            case 3:
                mode_res = capture.set(cv::CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv::CAP_OPENNI_QVGA_30HZ );
                break;
            case 4:
                mode_res = capture.set(cv::CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv::CAP_OPENNI_QVGA_60HZ );
                break;
            default:
                CV_Error( cv::Error::StsBadArg, "Unsupported image mode property.\n");
        }
        if (!mode_res)
            std::cout << "\nThis image mode is not supported by the device, the default value (CV_CAP_OPENNI_SXGA_15HZ) will be used.\n" << std::endl;
    }
    capture.set(cv::CAP_PROP_OPENNI2_MIRROR, false);
    // turn on depth, color and IR if needed
    if (retrived_img_flags != NULL) {
        if (retrived_img_flags[0] || retrived_img_flags[1] || retrived_img_flags[2])
            capture.set(cv::CAP_OPENNI_DEPTH_GENERATOR_PRESENT, true);
        else
            capture.set(cv::CAP_OPENNI_DEPTH_GENERATOR_PRESENT, false);
        if (retrived_img_flags[3] || retrived_img_flags[4])
            capture.set(cv::CAP_OPENNI_IMAGE_GENERATOR_PRESENT, true);
        else
            capture.set(cv::CAP_OPENNI_IMAGE_GENERATOR_PRESENT, false);
        if (retrived_img_flags[5])
            capture.set(cv::CAP_OPENNI_IR_GENERATOR_PRESENT, true);
        else
            capture.set(cv::CAP_OPENNI_IR_GENERATOR_PRESENT, false);
    } else {
        capture.set(cv::CAP_OPENNI_DEPTH_GENERATOR_PRESENT, true);
        capture.set(cv::CAP_OPENNI_IMAGE_GENERATOR_PRESENT, true);
    }


    // Print some avalible device settings.
    if (capture.get(cv::CAP_OPENNI_DEPTH_GENERATOR_PRESENT))
    {
        std::cout << "\nDepth generator output mode:" << std::endl <<
            "FRAME_WIDTH      " << capture.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl <<
            "FRAME_HEIGHT     " << capture.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl <<
            "FRAME_MAX_DEPTH  " << capture.get(cv::CAP_PROP_OPENNI_FRAME_MAX_DEPTH) << " mm" << std::endl <<
            "FPS              " << capture.get(cv::CAP_PROP_FPS) << std::endl <<
            "REGISTRATION     " << capture.get(cv::CAP_PROP_OPENNI_REGISTRATION) << std::endl;
    }
    else
    {
        std::cout << "\nDevice doesn't contain depth generator or it is not selected." << std::endl;
    }

    if( capture.get(cv::CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) )
    {
        std::cout <<
            "\nImage generator output mode:" << std::endl <<
            "FRAME_WIDTH   " << capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR+cv::CAP_PROP_FRAME_WIDTH ) << std::endl <<
            "FRAME_HEIGHT  " << capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR+cv::CAP_PROP_FRAME_HEIGHT ) << std::endl <<
            "FPS           " << capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR+cv::CAP_PROP_FPS ) << std::endl;
    }
    else
    {
        std::cout << "\nDevice doesn't contain image generator or it is not selected." << std::endl;
    }

    if( capture.get(cv::CAP_OPENNI_IR_GENERATOR_PRESENT) )
    {
        std::cout <<
            "\nIR generator output mode:" << std::endl <<
            "FRAME_WIDTH   " << capture.get(cv::CAP_OPENNI_IR_GENERATOR + cv::CAP_PROP_FRAME_WIDTH) << std::endl <<
            "FRAME_HEIGHT  " << capture.get(cv::CAP_OPENNI_IR_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT) << std::endl <<
            "FPS           " << capture.get(cv::CAP_OPENNI_IR_GENERATOR + cv::CAP_PROP_FPS) << std::endl;
    }
    else
    {
        std::cout << "\nDevice doesn't contain IR generator or it is not selected." << std::endl;
    }
    return true;
}

bool OpenNI2::getDepthMap(cv::Mat &depth) {
    if (!capture.grab()) {
        std::cout << "Can not grab images." << std::endl;
        return false;
    }
    bool res = capture.retrieve(depth, cv::CAP_OPENNI_DEPTH_MAP);
    return res;
}

bool OpenNI2::getValidDepthMap(cv::Mat &valid_depth) {
    if (!capture.grab()) {
        std::cout << "Can not grab images." << std::endl;
        return false;
    }
    bool res = capture.retrieve(valid_depth, cv::CAP_OPENNI_VALID_DEPTH_MASK);
    return res;
}
bool OpenNI2::getDisparityMap(cv::Mat &disparity) {
    if (!capture.grab()) {
        std::cout << "Can not grab images." << std::endl;
        return false;
    }
    bool res = capture.retrieve(disparity, cv::CAP_OPENNI_DISPARITY_MAP);
    return res;
}

bool OpenNI2::getBGRImage(cv::Mat &img) {
    if (!capture.grab()) {
        std::cout << "Can not grab images." << std::endl;
        return false;
    }
    bool res = capture.retrieve(img, cv::CAP_OPENNI_BGR_IMAGE);
    return res;
}

bool OpenNI2::getGrayImage(cv::Mat &gray) {
    if (!capture.grab()) {
        std::cout << "Can not grab images." << std::endl;
        return false;
    }
    bool res = capture.retrieve(gray, cv::CAP_OPENNI_GRAY_IMAGE);
    return res;
}

bool OpenNI2::getIRImage(cv::Mat &ir) {
    if (!capture.grab()) {
        std::cout << "Can not grab images." << std::endl;
        return false;
    }
    bool res = capture.retrieve(ir, cv::CAP_OPENNI_IR_IMAGE);
    return res;
}

bool OpenNI2::getImage(cv::Mat &img, int type) {
    if (!capture.grab()) {
        std::cout << "Can not grab images." << std::endl;
        return false;
    }
    bool res = capture.retrieve(img, type);
    return res;
}

bool OpenNI2::saveData(std::string &path, cv::Mat &img, ImgType type, ImgExt ext, int &index) {
    std::string file_name;
    char s_index[10];
    switch (type) {
    case ImgType::COLOR:
        file_name = "color-";
        break;
    case ImgType::GRAY:
        file_name = "gray-";
        break;
    case ImgType::DEPTH:
        file_name = "depth-";
        break;
    case ImgType::DISPARITY:
        file_name = "disaprity-";
        break;
    case ImgType::IR:
        file_name = "ir-";
        break;
    default:
        break;
    }
    std::sprintf(s_index, "%9d", index);
    file_name = file_name + std::string(s_index);

    switch (ext) {
    case ImgExt::JPG:
        file_name = file_name + ".jpg";
        break;
    case ImgExt::PNG:
        file_name = file_name + ".png";
        break;
    case ImgExt::BMP:
        file_name = file_name + ".bmp";
        break;
    default:
        break;
    }
    return true;
}

bool OpenNI2::holeLabeling(cv::Mat &disparity, std::vector<std::vector<cv::Point> > &regs) {
    uchar hole = 0, tmp_v;
    cv::Point p;
    std::vector<cv::Point> reg;
    regs.clear();
    reg.push_back(cv::Point(0,0));
    for (int y = 1; y < disparity.rows; y++) {
        for (int x = 1; x < disparity.cols -1; x++) {
            p.x = x;
            p.y = y;
            tmp_v = disparity.at<uchar>(p);
            if (tmp_v != hole) {
                this->floodFill(reg, disparity, p);
                if (reg.size() > 40)
                regs.push_back(reg);
            }
        }
    }
    regs.shrink_to_fit();
    return true;
}

bool OpenNI2::floodFill(std::vector<cv::Point> &res, cv::Mat &image, cv::Point &p) {
    //    int height, width;
    uchar hole = 0;
    uchar white = 0;;
    uchar tmp;
    cv::Point n;
    cv::Point new_point;
    res.clear();

    if (image.at<uchar>(p) == hole) return false;
    std::vector<cv::Point> q;
    q.push_back(p);
    while (q.size() != 0) {
        n = q[0];
        q.erase(q.begin());
        if (image.at<uchar>(n) != hole) {
            image.at<uchar>(n) = white;
            res.push_back(n);

            new_point.x = n.x + 1;
            if (new_point.x <image.cols) {
                new_point.y = n.y;
                tmp = image.at<uchar>(new_point);
                if (tmp != hole) q.push_back(new_point);
            }

            new_point.x = n.x - 1;
            if (new_point.x >= 0) {
                new_point.y = n.y;
                tmp = image.at<uchar>(new_point);
                if (tmp != hole) q.push_back(new_point);
            }

            new_point.y = n.y + 1;
            if (new_point.y <image.rows) {
                new_point.x = n.x;
                tmp = image.at<uchar>(new_point);
                if (tmp != hole) q.push_back(new_point);
            }

            new_point.y = n.y - 1;
            if (new_point.y >= 0) {
                new_point.x = n.x;
                tmp = image.at<uchar>(new_point);
                if (tmp != hole) q.push_back(new_point);
            }

            new_point.x = n.x + 1;
            new_point.y = n.y + 1;
            if ((new_point.x < image.cols) && (new_point.y < image.rows)) {
                tmp = image.at<uchar>(new_point);
                if (tmp != hole) q.push_back(new_point);
            }

            new_point.x = n.x + 1;
            new_point.y = n.y - 1;
            if ((new_point.x < image.cols) && (new_point.y >= 0)) {
                tmp = image.at<uchar>(new_point);
                if (tmp != hole) q.push_back(new_point);
            }

            new_point.x = n.x - 1;
            new_point.y = n.y + 1;
            if ((new_point.x >= 0) && (new_point.y < image.rows)) {
                tmp = image.at<uchar>(new_point);
                if (tmp != hole) q.push_back(new_point);
            }

            new_point.x = n.x - 1;
            new_point.y = n.y - 1;
            if ((new_point.x >= 0) && (new_point.y >= 0)) {
                tmp = image.at<uchar>(new_point);
                if (tmp != hole) q.push_back(new_point);
            }
        }
    }
    return true;
}



bool OpenNI2::drawLabeledHole(std::vector<std::vector<cv::Point> > &regs, cv::Mat &image) {
    std::vector<std::vector<cv::Point> >::iterator it;
    std::vector<cv::Point>::iterator it_point;
    std::vector<cv::Point> reg;
    cv::Vec3b color;
    cv::Point p;
    size_t m_index = 0;
    int m = 0;
    for (it = regs.begin(); it != regs.end(); ++it) {
        reg = *it;
        m = m_index %10;
        switch (m) {
        case 0:
            color = cv::Vec3b(255,0,0);
            break;
        case 1:
            color = cv::Vec3b(0,255,0);
            break;
        case 2:
            color = cv::Vec3b(0,0,255);
            break;
        case 3:
            color = cv::Vec3b(255,0,255);
            break;
        case 4:
            color = cv::Vec3b(255,255,0);
            break;
        case 5:
            color = cv::Vec3b(0,255,255);
            break;
        case 6:
            color = cv::Vec3b(175,0,0);
            break;
        case 7:
            color = cv::Vec3b(0,175,0);
            break;
        case 8:
            color = cv::Vec3b(0,0,175);
            break;
        default:
            color = cv::Vec3b(175,0,175);
            break;
        }

        for (it_point = reg.begin(); it_point != reg.end(); ++it_point) {
            p = *it_point;
            image.at<cv::Vec3b>(p) = color;
        }

        m_index ++;
    }
    return true;
}





