#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
using namespace std;

int main( int argc, char** argv )
{
    // data fetch
    cv::Mat rgb, depth;
    rgb = cv::imread("../data/rgb.png");
    depth = cv::imread("../data/depth.png", -1);
    
    // Zoom factor in camera 
    double cx = 325.4782409667969;
    double cy =  177.57662963867188;
    double fx = 461.0956115722656;
    double fy = 460.9181823730469;
    double depthScale = 1000.0;
    
    // Define the format used by the point cloud: XYZRGB is used here
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // Calculate the XYZRGB value corresponding to each point
    PointCloud::Ptr pointCloud(new PointCloud);
    for ( int v=0; v<rgb.rows; v++ )
        for (int u=0; u<rgb.cols; u++)
        {
            unsigned int d = depth.ptr<unsigned short>(v)[u];
            if (d==0)
                continue;
            PointT p;
            p.z = double(d)/depthScale;
            p.x = (u-cx)*p.z/fx;
            p.y = (v-cy)*p.z/fy;
            p.b = rgb.data[v*rgb.step+u*rgb.channels()];
            p.g = rgb.data[v*rgb.step+u*rgb.channels()+1];
            p.r = rgb.data[v*rgb.step+u*rgb.channels()+2];
            pointCloud->points.push_back(p);
        }
        
    // Point cloud save
    pointCloud->is_dense = false;
    pointCloud->width = pointCloud->points.size();
    pointCloud->height = 1;
    fprintf(stderr, "%d, %d\n", rgb.cols, rgb.rows);
    cout << "Common cloud point" << pointCloud->size() << "Point." << endl;
    pcl::io::savePCDFileASCII("../data/0000.pcd", *pointCloud );
    return 0;
}