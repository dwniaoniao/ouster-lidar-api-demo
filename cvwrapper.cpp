#include <iostream>
#include <opencv2/opencv.hpp>

template<typename T>
void display_image(T *buf, size_t width, size_t height)
{
    cv::Mat img_temp(height, width, CV_32SC1);
    T (*p)[width] = reinterpret_cast<T (*)[width]>(buf);
    for(size_t i = 0; i < height; i++){
        for(size_t j = 0; j < width; j++){
            img_temp.at<int>(i, j) = p[i][j];
        }
    }
    cv::Mat img_0_255;
    cv::normalize(img_temp, img_0_255, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat img_bgr;
    cv::applyColorMap(img_0_255, img_bgr, cv::COLORMAP_TURBO);
    cv::imshow("image", img_bgr);
}

extern "C"{

void create_window()
{
    cv::namedWindow("image", cv::WINDOW_NORMAL);
}

void destroy_all_windows()
{
    cv::destroyAllWindows();
}

int wait_key(int delay)
{
    return cv::waitKey(delay);
}

void display_image_8u(uint8_t *buf, size_t width, size_t height)
{
    display_image(buf, width, height);
}

void display_image_16u(uint16_t *buf, size_t width, size_t height)
{
    display_image(buf, width, height);
}

void display_image_32u(uint32_t *buf, size_t width, size_t height)
{
    display_image(buf, width, height);
}

}

