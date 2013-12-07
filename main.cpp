#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <time.h>
#include <cstdlib>

void findRectangles(const std::vector<std::vector<cv::Point> > & contours, std::vector<std::vector<cv::Point> > & rectangles) {
    rectangles.clear();
    for (std::vector<std::vector<cv::Point> >::const_iterator it = contours.begin(); it != contours.end(); it++) {
        std::vector<cv::Point> contour = *it;
        double minX = 1e6;
        double maxX = -1e6;
        double minY = 1e6;
        double maxY = -1e6;
        for (std::vector<cv::Point>::iterator pit = contour.begin(); pit != contour.end(); pit++) {
            cv::Point point = *pit;
            minX = point.x < minX ? point.x : minX;
            maxX = point.x > maxX ? point.x : maxX;
            minY = point.y < minY ? point.y : minY;
            maxY = point.y > maxY ? point.y : maxY;
        }
        std::vector<cv::Point> rect;
        rect.push_back(*(new cv::Point(minX, minY)));
        rect.push_back(*(new cv::Point(minX, maxY)));
        rect.push_back(*(new cv::Point(maxX, maxY)));
        rect.push_back(*(new cv::Point(maxX, minY)));
        rectangles.push_back(rect);
    }

}

void filterSmall(std::vector<std::vector<cv::Point> > & rectangles, double minArea) {
    std::vector<std::vector<cv::Point> >::iterator it = rectangles.begin();
    while (it != rectangles.end()) {
        std::vector<cv::Point> rect = *it;
        double xrange = rect[2].x - rect[0].x;
        double yrange = rect[1].y - rect[0].y;
        if (xrange * yrange < minArea) {
            it = rectangles.erase(it);
        } else {
            it++;
        }

    }
}

int main(int argc, char *argv[])
{
    time_t now, prev;
    time(&now);
    time(&prev);
    std::cout<<"Entering crossroad"<<std::endl;
    std::string * filename = 0;
    if (argc > 1)
    {
        filename = new std::string(argv[1]);
    }
    

    cv::Mat frame;
    cv::Mat back;
    cv::Mat fore;
    cv::VideoCapture * capture;
    //capture->set(CV_CAP_PROP_FRAME_WIDTH,64);
    //capture->set(CV_CAP_PROP_FRAME_HEIGHT,64);
    

    if (filename != NULL)
    {
        std::cout<<"Reading from "<<*filename<<std::endl;
        capture = new cv::VideoCapture(*filename);

    }
    else
    {
        std::cout<<"Reading from default camera"<<std::endl;
        capture = new cv::VideoCapture(0);
        capture->set(CV_CAP_PROP_FRAME_WIDTH, 160);
        capture->set(CV_CAP_PROP_FRAME_HEIGHT, 120);
    }

    if (!capture->isOpened())
    {
        std::cout<<"Could not open video source\n";
        return -1;
    }
    cv::BackgroundSubtractorMOG2 bg(3, 16, true);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > rectangles;

    cv::namedWindow("Frame",  CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("Background",  CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("Foreground", CV_WINDOW_AUTOSIZE);

    while (true)
    {
        *capture >> frame;
        if (frame.empty()) {
            std::cout<<"Frame empty"<<std::endl;
        }
        bg(frame, fore, 0.01);
        bg.getBackgroundImage(back);
        cv::erode(fore, fore, cv::Mat());
        cv::dilate(fore, fore, cv::Mat());
        cv::findContours(fore, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
        findRectangles(contours, rectangles);
        filterSmall(rectangles, 160 * 120);
        cv::drawContours(frame, rectangles, -1, cv::Scalar(0, 0, 255), 2);
        cv::imshow("Frame", frame);
        //cv::imshow("Background", back);
        //cv::imshow("Foreground", fore);

        if (cv::waitKey(30) >= 0) break;
        time(&now);
        double sec = difftime(now, prev);
        time(&prev);
        std::cout<<1./sec<<" fps"<<std::endl;
    }

    if (filename != NULL)
    {
        delete filename;
    }
    delete capture;
    return 0;
}
