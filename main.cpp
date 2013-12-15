#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <time.h>
#include <cstdlib>
#include <sys/time.h>

class TraceEntry {
    public:
        cv::Point point;
        timeval time;
};

class Trackable {
    private:
        int traceTTL;
        std::deque<TraceEntry*> trace;

        int getTimeDifferenceInMS(const struct timeval & lhs, const struct timeval & rhs) {
            int difference = 0;
            difference = (lhs.tv_sec - rhs.tv_sec) * 1000;
            if (difference < 0) {
                difference = -difference;
            }
            int usecdiff = lhs.tv_usec - rhs.tv_usec;
            if (usecdiff < 0) {
                usecdiff = -usecdiff;
            }
            difference += usecdiff / 1000;
            return difference;
        }

    public:
        Trackable() : traceTTL(1000) {
        }

        Trackable(int traceTTL) : traceTTL(traceTTL) {
        }

        ~Trackable() {
            while (!trace.empty()) {
                delete trace.front();
                trace.erase(trace.begin());
            }
        }

        void setTraceTTL(int ms) {
            traceTTL = ms;
        }

        int getTraceTTL() {
            return traceTTL;
        }

        cv::Point getLastLocalization() {
            if (!trace.empty()) {
                return trace.back()->point;
            }
            return *(new cv::Point(0, 0));
        }

        void signal(cv::Point point) {
            timeval t;
            gettimeofday(&t, NULL);
            TraceEntry * entry = new TraceEntry;
            entry->point = point;
            entry->time = t;
            trace.push_back(entry);
        }

        void refresh() {
            timeval now;
            gettimeofday(&now, NULL);
            while(!trace.empty()) {
                TraceEntry * lastEntry = trace.front();
                if (getTimeDifferenceInMS(now, lastEntry->time) > traceTTL) {
                    trace.pop_front();
                    delete lastEntry;
                    std::cout<<"Removed last entry of a trace"<<std::endl;
                } else {
                    return;
                }
            }
        }

        bool hasAnyPoints() {
            return trace.empty();
        }

        std::vector<cv::Point> getTracePoints() {
            std::vector<cv::Point> ret;
            for (std::deque<TraceEntry*>::iterator it = trace.begin(); it != trace.end(); it++) {
                TraceEntry * entry = *it;
                ret.push_back(entry->point);
            }
            return ret;
        }

};

class ObjectTracker {
    private:
        int traceTTL;
        double distanceThreshold;
        std::deque<Trackable*> objects;

        void refresh() {
            while (!objects.empty()) {
                Trackable * trackable = objects.front();
                trackable->refresh();
                if (!trackable->hasAnyPoints()) {
                    objects.pop_front();
                    std::cout<<"Removed object with no points left"<<std::endl;
                } else {
                    return;
                }
                delete trackable;
            }
        }

        double getDistance(cv::Point lhs, cv::Point rhs) {
            double xDist = lhs.x - rhs.x;
            double yDist = rhs.y - rhs.y;
            return xDist*xDist + yDist*yDist;
        }

        std::vector<std::vector<cv::Point > >::iterator findClosest(cv::Point point, std::vector<std::vector<cv::Point> > & contours) {
            std::vector<std::vector<cv::Point> >::iterator closest = contours.end();
            double minDistance = 1e9;
            for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); it++) {
                cv::Point cog = getCOG(*it);
                double distance = getDistance(cog, point);
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = it;
                }
            }
            return closest;
        }

        cv::Point getCOG(const std::vector<cv::Point> & contour) {
            cv::Point ret;
            ret.x = 0;
            ret.y = 0;
            for (std::vector<cv::Point>::const_iterator it = contour.begin(); it != contour.end(); it++) {
                ret.x += (*it).x;
                ret.y += (*it).y;
            }
            ret.x /= contour.size();
            ret.y /= contour.size();
            return ret;
        }

        void appendToExisting(std::vector<std::vector<cv::Point> > & contours) {
            for (std::deque<Trackable*>::iterator it = objects.begin(); it != objects.end(); it++) {
                Trackable * trackable = *it;
                std::vector<std::vector<cv::Point > >::iterator closest = findClosest(trackable->getLastLocalization(), contours);
                if (closest != contours.end()) {
                    std::vector<cv::Point> contour = *closest;
                    cv::Point cog = getCOG(contour);
                    if (getDistance(cog, trackable->getLastLocalization()) < distanceThreshold) {
                        trackable->signal(cog);
                        contours.erase(closest);
                    }
                }
            }
        }

        void createNew(std::vector<std::vector<cv::Point> > & contours) {
            for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); it++) {
                Trackable * trackable = new Trackable(traceTTL);
                cv::Point cog = getCOG(*it);
                trackable->signal(cog);
                objects.push_back(trackable);
            }
        }

    public:
        ObjectTracker(int traceTTL, double distanceTreshold) : traceTTL(traceTTL), distanceThreshold(distanceTreshold) {
        }

        ~ObjectTracker() {
            while (!objects.empty()) {
                delete objects.front();
                objects.erase(objects.begin());
            }
        }

        void setTraceTTL(int ms) {
            traceTTL = ms;
        }

        int getTraceTTL() {
            return traceTTL;
        }

        void setDistanceTreshold(double distance) {
            distanceThreshold = distance;
        }

        double getDistanceTreshold() {
            return distanceThreshold;
        }

        void record(std::vector<std::vector<cv::Point> > & contours) {
            refresh();
            appendToExisting(contours);
            createNew(contours);
        }

        const std::deque<Trackable*> & getTracked() {
            return objects;
        }

        int getTrackedCount() {
            return objects.size();
        }

};

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

void drawTraces(cv::Mat & img, ObjectTracker * objectTracker) {
    std::deque<Trackable*> tracked = objectTracker->getTracked();
    if (tracked.empty()) {
        return;
    }
    cv::Point ** curves = new cv::Point *[tracked.size()];
    int * curveLengths = new int[tracked.size()];
    int cid = 0;
    for (std::deque<Trackable*>::iterator it = tracked.begin(); it != tracked.end(); it++) {
        Trackable * trackable = *it;
        std::vector<cv::Point> trace = trackable->getTracePoints();
        curves[cid] = new cv::Point[trace.size()];
        for (int i = 0; i < trace.size(); i++) {
            curves[cid][i].x = trace[i].x;
            curves[cid][i].y = trace[i].y;
        }
        curveLengths[cid] = trace.size();
        std::cout<<"Current pos: "<<trackable->getLastLocalization()<<" Trace length: "<<trace.size()<<std::endl;
        cid++;
    }
    cv::polylines(img, (const cv::Point **) curves, curveLengths, cid, false, cvScalar(0, 255, 255), 2);
    for (int i = 0; i < cid; i++) {
        delete[] curves[i];
    }
    delete[] curves;
    delete[] curveLengths;
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

    ObjectTracker * objectTracker = new ObjectTracker(1000, 100000);

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
        filterSmall(rectangles, 100 * 100);
        objectTracker->record(rectangles);
        cv::drawContours(frame, rectangles, -1, cv::Scalar(0, 0, 255), 2);
        drawTraces(frame, objectTracker);
        cv::imshow("Frame", frame);
        //cv::imshow("Background", back);
        //cv::imshow("Foreground", fore);

        if (cv::waitKey(30) >= 0) break;
        time(&now);
        double sec = difftime(now, prev);
        time(&prev);
        std::cout<<1./sec<<" fps"<<std::endl;
        std::cout<<"Total objects present: "<<objectTracker->getTrackedCount()<<std::endl;
    }

    if (filename != NULL)
    {
        delete filename;
    }
    delete capture;
    delete objectTracker;
    return 0;
}
