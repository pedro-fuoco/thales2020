#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h> 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "thales2020/H_info.h"
#include "std_msgs/Bool.h"

#define ANGLE_THRESH 0.01
#define PI 3.14159265

#define vp vector<Point>
#define vpf vector<Point2f>

// False for default, since this is the ROS algorithm
#define DEBUG false
/*////////////////////////////////////////////////////////////////////////////////////
THE COMMENTS ABOUT THE ALGORITHM ARE PRESENTED IN h_sem_ros.cpp FOR DIDATIC REASONS
IN THIS CODE ONLY THE ROS IMPLEMENTATION IS DETAILED
/*////////////////////////////////////////////////////////////////////////////////////

struct comparison {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y);}
} comparing;

class HDetector {
    private:
        vpf edge_pts = { Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0) };
        Rect bounds;
        float area_ratio;
        void order_points();
        Mat four_points_transform(Mat image);
        float angle(Point2f v1, Point2f v2, Point2f relative = Point2f(0,0) );
        bool angle_check(vpf pts);
        ros::NodeHandle n;
        void image_cb(const sensor_msgs::ImageConstPtr& img);
        void runnin_state_cb(std_msgs::Bool data);
        ros::Publisher h_pub;
        ros::Publisher img_debug_pub;
        ros::Subscriber h_sub_image;
        ros::Subscriber h_sub_runner;
        bool runnin;
    public:
        Mat warped; 
        HDetector();
        bool detect (Mat frame);
        float getArea();
        void setArea(vp contour, Mat frame);
        int getCenter_X();
        int getCenter_Y();
};

HDetector::HDetector(){
    //Here, the ros topics are matched with their subscribers and publishers, respectively
    this->h_pub = this->n.advertise<thales2020::H_info>("/cv_detection/detection", 0);
    this->img_debug_pub = this->n.advertise<sensor_msgs::Image>("/cv_detection/debug/image_raw", 0);
    this->h_sub_image = this->n.subscribe("/iris_fpv_cam/usb_cam/image_raw", 5, &HDetector::image_cb, this);
    this->h_sub_runner = this->n.subscribe("/cv_detection/set_running_state",10, &HDetector::runnin_state_cb, this);
}

void HDetector::order_points(){

    sort(this->edge_pts.begin(), this->edge_pts.end(), comparing);
    Point2f p1, p2;

    if(this->edge_pts[0].x > this->edge_pts[1].x ){
        p1 = this->edge_pts[1];
        this->edge_pts[1] = this->edge_pts[0];
        this->edge_pts[0] = p1;
    }

    if(this->edge_pts[2].x < this->edge_pts[3].x){
        p2 = this->edge_pts[3];
        this->edge_pts[3] = this->edge_pts[2];
        this->edge_pts[2] = p2;
    }

}

void HDetector::runnin_state_cb(std_msgs::Bool data){
    this->runnin = data.data;
}

void HDetector::image_cb(const sensor_msgs::ImageConstPtr& img){
    if(this->runnin){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        thales2020::H_info msg;

        if (this->detect(cv_ptr->image)){
            msg.detected = true;
            msg.center_x = this->getCenter_X();
            msg.center_y = this->getCenter_Y();
            msg.area_ratio = this->getArea();
            this->h_pub.publish(msg);
        }
    }
}

Mat HDetector::four_points_transform(Mat image){

    order_points();

    Point tl = this->edge_pts[0];
    Point tr = this->edge_pts[1];
    Point br = this->edge_pts[2];
    Point bl = this->edge_pts[3];

    float widthA =  sqrt( abs ( pow( br.x - bl.x, 2.0) - pow( br.y - bl.y, 2.0 ) ) );  
    float widthB =  sqrt( abs ( pow( tr.x - tl.x, 2.0) - pow( tr.y - tl.y, 2.0 ) ) );
    // Requires explicit reference due to cv::max
    float maxWidth = std::max(widthA, widthB);

    float heightA =  sqrt( abs ( pow( br.y - tr.y, 2.0 ) - pow( br.x - tr.x, 2.0) ) ); 
    float heightB =  sqrt( abs ( pow( tl.y - bl.y, 2.0 ) - pow( tl.x - bl.x, 2.0) ) );

    float maxHeight = std::max(heightA, heightB);

    vpf dst = {
        Point2f(0.0, 0.0) ,
        Point2f(maxWidth, 0.0) ,
        Point2f(maxWidth, maxHeight),
        Point2f(0.0, maxHeight)
    };

    Mat M = getPerspectiveTransform(this->edge_pts, dst);
    
    warpPerspective(image, this->warped, M, Size(maxWidth, maxHeight));

    return M;
}


float HDetector::angle(Point2f v1, Point2f v2, Point2f relative){
    
    float Vlenght1, Vlenght2;
    
    Vlenght1 = sqrt((v1.x - relative.x)*(v1.x - relative.x) + (v1.y - relative.y)*(v1.y - relative.y));
    Vlenght2 = sqrt((v2.x - relative.x)*(v2.x - relative.x) + (v2.y - relative.y)*(v2.y - relative.y));
    float a = ((v1.x - relative.x)*(v2.y - relative.y) - (v1.y - relative.y)*(v2.x -relative.x))/(Vlenght1*Vlenght2);

    return asin(a);
}

bool HDetector::angle_check(vpf pts){
    
    int bitmasks[8] = {2145,195,390,780,1560,3120};
    int current_bm = 0;

    float a = 0;
    for (int i = 0; i < 12; i++){
        
        a = angle(pts[(12+i+1)%12], pts[(12+i-1)%12], pts[(12+i)%12]);
        if ( abs( abs(a) - PI/2 ) < ANGLE_THRESH)
            return false;
        else
            current_bm = current_bm | (a > 0) << i;
    }

    for(int bm : bitmasks){
        if(current_bm == bm) 
            return true;
    }
    return false;

}

float HDetector::getArea(){
    return this->area_ratio;
}

//This function divides the contour area of the H by the total size of the screen,
//that allows us to approximate how close we are to the H
void HDetector::setArea(vp contour, Mat frame){
    this->area_ratio = contourArea(contour, false)/(frame.cols*frame.rows);
}

//returns the coordinate in X of the H center
int HDetector::getCenter_X(){
    return (this->bounds.x + this->bounds.width/2);
}

//returns the coordinate in Y of the H center
int HDetector::getCenter_Y(){
    return (this->bounds.y + this->bounds.height/2);
}


bool HDetector::detect (Mat frame){
    Mat frame2 = frame;
    bool detected = false;
    cvtColor(frame, frame, CV_BGR2GRAY);
    threshold(frame, frame, 115, 255, 1);
    adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 9, 20.0);
    adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 0.0);
    
    vector<vp> contour;
    findContours(frame, contour, RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for(vp cnt : contour){

        int peri = arcLength(cnt, true);
        vpf approx;
        approxPolyDP(cnt, approx, 0.02*peri, true);
        
        if (approx.size() == 12){
            
            this->bounds = boundingRect(approx);
            
            float a1 = angle(approx[0] - approx[1], Point2f(0,1));
            float a2 = angle(approx[1] - approx[2], Point2f(0,1));

            if( a1 < 0.1 || a2 < 0.1
               || abs(a1 - PI) < 0.1 || abs(a1 - PI) < 0.1 ){

                this->edge_pts = {
                    Point2f (bounds.x, bounds.y) ,
                    Point2f (bounds.x + bounds.width, bounds.y) , 
                    Point2f (bounds.x, bounds.y + bounds.height) ,
                    Point2f (bounds.x + bounds.width, bounds.y + bounds.height)
                };
            
            }else{
                for(Point2f v : approx){
                    if( abs(v.x - bounds.x) <= 1) this->edge_pts[0] = v;
                    else if( abs(v.x - (bounds.x + bounds.width) ) <= 1) this->edge_pts[1] = v;
                    else if( abs(v.y - bounds.y) <= 1) this->edge_pts[2] = v;
                    else if( abs(v.y - (bounds.y + bounds.height) ) <= 1) this->edge_pts[3] = v;
                }
            }
            Mat perspective = four_points_transform(frame);
            vpf transformed;
            perspectiveTransform(approx, transformed, perspective);

            if (angle_check(approx)){
                if(DEBUG){ 
                    imshow("warped", frame);
                    circle(frame2, this->edge_pts[0], 3, (255,0,0), 3 );
                    circle(frame2, this->edge_pts[1], 3, (255,0,0), 3 );
                    circle(frame2, this->edge_pts[2], 3, (255,0,0), 3 );                
                    circle(frame2, this->edge_pts[3], 3, (255,0,0), 3 );
                    rectangle(frame2, bounds, (0,255,0));
                    imshow("Lines", frame2);
                    cout << "H detectado!" << endl;
                }
                detected = true;
            }
        }
    }
    return detected;
}

// For testing
int main(int argc, char** arvg){
    ROS_INFO("Running H detection node!");
    ros::init(argc, arvg, "h_node");
    HDetector* detector = new HDetector();
    ros::spin();
}

