#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include "Triangulation.h"
#include "Common.h"
#include "Visualization.h"
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;
using namespace cv;


typedef struct PointCoord {
    Point2d coord_real;
    Point2d coord_norm;
} PointCoord;

typedef struct Observation {
    int camera;
    PointCoord coord;
} Observation;



void getRandTfromE(const Mat& E, Mat& R, Mat& t, bool inv_w = false) {
    // SVD decomposition
    SVD svd = SVD(E);
    Mat U = svd.u;
    Mat V_t = svd.vt;
    Mat V = V_t.t();
    double W_t_data[3][3] = { {0, 1, 0},
                            {-1,  0, 0},
                            {0,  0, 1} };

    Mat W_t = Mat(3, 3, CV_64F, &W_t_data);

    double Z_data[3][3] = { {0, -1, 0},
                            {1,  0, 0},
                            {0,  0, 0} };

     Mat Z = Mat(3, 3, CV_64F, &Z_data);

    if (inv_w) {
        W_t = W_t.inv();
    }

    // tx : tx = V *  Z * V^T
    Mat tx_tmp = V * Z * V_t;

    Mat tx = Mat(3, 1, CV_64F);
    tx.at<double>(0, 0) = tx_tmp.at<double>(2, 1);
    tx.at<double>(1, 0) = tx_tmp.at<double>(0, 2);
    tx.at<double>(2, 0) = tx_tmp.at<double>(1, 0);

    // R = U * W^T * V^T

    R = U * W_t * V_t;
    t = U.col(2);


    cout << "translation" << endl;
    cout << t << endl;
    cout << tx << endl;
}


bool visibleInBoth(Point3d point, Mat proj1, Mat proj2) {
        Mat P4x4 = Mat::eye(4, 4, CV_64F);
        for (int u = 0; u < 3; ++u) {
            for (int v = 0; v < 3; ++v) {
                P4x4.at<double>(u, v) = proj1.at<double>(u, v);
            }
        }
        
        vector<Point3d> in;
        vector<Point3d> out;
        in.push_back(point);
        perspectiveTransform(in, out, P4x4);
        Point3d p1 = out.back();

        out.clear();

        P4x4 = Mat::eye(4, 4, CV_64F);
        for (int u = 0; u < 3; ++u) {
            for (int v = 0; v < 3; ++v) {
                P4x4.at<double>(u, v) = proj2.at<double>(u, v);
            }
        }
        perspectiveTransform(in, out, P4x4);
        Point3d p2 = out.back();
        cout << "|||||||||||||||  " << p1.z << " ||||||||||| " << p2.z << " ||||||| " << endl << endl;
        return (p1.z+0.01 >= 0 && p2.z+0.01 >= 0);

}

bool checkGoodSolution(Point2d point1, Point2d point2, Mat cameraMatrix1, Mat cameraMatrix2, Mat R, Mat t) {
    Mat projMat1, projMat2;
    hconcat(Mat::eye(3, 3, CV_64F), Mat::zeros(3, 1, CV_64F), projMat1);
    projMat1 = cameraMatrix1 * projMat1;
    hconcat(R, t, projMat2);
    projMat2 = cameraMatrix2 * projMat2;

    Mat_<double> testp = IterativeLinearLSTriangulation(Point3d(point1.x, point1.y, 1), projMat1, Point3d(point2.x, point2.y, 1), projMat2);

    return (visibleInBoth(Point3d(testp.at<double>(0, 0), testp.at<double>(0, 1), testp.at<double>(0, 2)), projMat1, projMat2));
}

bool selectGoodSolution(Point2d point1, Point2d point2, Mat E, Mat cameraMatrix1, Mat cameraMatrix2, Mat& R, Mat& t) {

    getRandTfromE(E, R, t);
    if (determinant(R) < 0) {
         getRandTfromE(-E, R, t);
    }

    if (!checkGoodSolution(point1, point2, cameraMatrix1, cameraMatrix2, R, t)) {
        if (!checkGoodSolution(point1, point2, cameraMatrix1, cameraMatrix2, R, -t)) {

            getRandTfromE(E, R, t, true);
            if (determinant(R) < 0) {
                 getRandTfromE(-E, R, t, true);
            }

            if (!checkGoodSolution(point1, point2, cameraMatrix1, cameraMatrix2, R, t)) {
                if (!checkGoodSolution(point1, point2, cameraMatrix1, cameraMatrix2, R, -t)) {
                    return false;
                }
                else {
                    t = -t;
                }

            }

        }
        else {
            t = -t;
        }
    }

    return true;
}

Vec3d triangulatePoints(Vec2d point1, Vec2d point2, CvMat proj1, CvMat proj2) {
    CvMat *p1 = cvCreateMat(2, 1, CV_64F);
    CvMat *p2 = cvCreateMat(2, 1, CV_64F);
    CV_MAT_ELEM( *p1, double, 0, 0 ) = point1[0];
    CV_MAT_ELEM( *p1, double, 1, 0 ) = point1[1];
    CV_MAT_ELEM( *p2, double, 0, 0 ) = point2[0];
    CV_MAT_ELEM( *p2, double, 1, 0 ) = point2[1];

    CvMat *point3D = cvCreateMat(4, 1, CV_64F);

    cvTriangulatePoints(&proj1, &proj2, p1, p2, point3D);

    //to get the real position we need to do also a homogeneous division
    point3D->data.db[0] /= point3D->data.db[3];
    point3D->data.db[1] /= point3D->data.db[3];
    point3D->data.db[2] /= point3D->data.db[3];

    return Vec3d(point3D->data.db[0], point3D->data.db[1], point3D->data.db[2]);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "usage: " << argv[0] << " <number_of_camera> <dir_with_intrisic_param> <dir_with_tracking_result>" << endl;
        return -1;
    }

    int nr_of_camera = atoi(argv[1]);
    string dir_intrinsic = argv[2];
    string dir_tracking = argv[3];

    Mat cameraMatrix[nr_of_camera];
    Mat distCoeffs[nr_of_camera];

    map<int, vector<Observation> > final_points;
    int number_of_observation = 0;

    map<int, PointCoord> points_red[nr_of_camera];
    map<int, PointCoord> points_green[nr_of_camera];


    // For each camera we get the intrinsic parameters and the tracked points
    for (int i = 0; i < nr_of_camera; ++i) {
        ostringstream oss;
        oss << dir_intrinsic << "/out_camera_data_" << (i + 1) << ".xml";
        FileStorage fs(oss.str(), FileStorage::READ);
        if (fs.isOpened()) {
            fs["Camera_Matrix"] >> cameraMatrix[i];
            fs["Distortion_Coefficients"] >> distCoeffs[i];
            cout << "camera matrix: " << cameraMatrix[i] << endl << "distortion coeffs: " << distCoeffs[i] << endl;
        }
        fs.release();

        oss.str("");
        oss << dir_tracking << "/tracking_data_red_" << (i + 1) << ".txt";
        ifstream file;
        file.open(oss.str().c_str());
        if (file.is_open())
        {
            while (file.good())
            { 
                int frame, x, y;
                float radius;
                file >> frame >> x >> y >> radius;
                vector<Point2d> tmp;
                vector<Point2d> out;
                tmp.push_back(Point2d(x, y));
                undistortPoints(tmp, out, cameraMatrix[i], distCoeffs[i]);
                Point2d pundistort = tmp.back();
                points_red[i][frame] = { Point2d(x, y), Point2d(pundistort.x, pundistort.y) };
            }
            cout << points_red[i].size() << " red points loaded for camera " << i << endl;
            file.close();
        }

        oss.str("");
        oss << dir_tracking << "/tracking_data_green_" << (i + 1) << ".txt";
        file.open(oss.str().c_str());
        if (file.is_open())
        {
            while (file.good())
            { 
                int frame, x, y;
                float radius;
                file >> frame >> x >> y >> radius;
                //points_green[i][frame] = Vec3f(x, y, radius);
            }
            cout << points_green[i].size() << " green points loaded for camera " << i << endl;
            file.close();
        }

    }

    // Compute pairwise fundamental matrices

    cout << endl << endl << "############# COMPUTING FUNDAMENTAL MATRICES #############" << endl;
    Mat fundamentals[nr_of_camera - 1];
    Mat essentials[nr_of_camera - 1];

    Mat relative_transformation[nr_of_camera - 1][2];

    for (int i = 0; i < nr_of_camera - 1; ++i) {
        vector<Point2d> points1;
        vector<Point2d> points2;
        int commun = 0;

        for (auto it : points_red[i]) {
            if (points_red[i+1].count(it.first) > 0) {
                commun++;
                points1.push_back(it.second.coord_real);
                points2.push_back(points_red[i + 1][it.first].coord_real);
                final_points[it.first].push_back({i, it.second});
                final_points[it.first].push_back({i+1, points_red[i + 1][it.first]});
                number_of_observation+=2;
            }
        }

        vector<uchar> status;
        fundamentals[i] = findFundamentalMat(points1, points2, status);
        cout << "points correspondances : " << commun << endl;
        cout << "points kept : " << countNonZero(status) << endl;
        cout << "matrix found : " << endl << fundamentals[i] << endl;


        for (unsigned int j = 0; j < points1.size(); j++) {
            Mat p = Mat(Vec3d(points1[j].x, points1[j].y, 1));
            Mat p_2 = Mat(Vec3d(points2[j].x, points2[j].y, 1));
            p.convertTo(p, CV_64F);
            p_2.convertTo(p_2, CV_64F);
            Mat check = p_2.t() * fundamentals[i] * p;

            if (check.at<double>(0, 0)  <= -1 || check.at<double>(0, 0)  >= 1) {
                //cout << "PROBLEM DURING PAIRWISE TRANSFORMATION WITH CAMERAS " << i << " ";
                //cout << (i+1) << " not close to 0 : " << check.at<double>(0, 0) << endl;
            }
        }

        // getting the essential matrix:
        // E = K'^T * F * K

        essentials[i] = cameraMatrix[i+1].t() * fundamentals[i] * cameraMatrix[i];

        if (fabsf(determinant(essentials[i])) - 1.0 > 1e-07) {
            cerr << "det(E) != 0 : " << determinant(essentials[i]) << endl;
            exit(EXIT_FAILURE);
        }

        cout << "ESSENTIAL MATRIX " << endl << determinant(essentials[i]) << endl;
        cout << essentials[i] << endl;

        Mat R(3, 3, CV_64F);
        Mat t(3, 1, CV_64F);
        getRandTfromE(essentials[i], R, t);
        if (determinant(R) < 0) {
            getRandTfromE(-essentials[i], R, t);
        }

        // Find the right solution among the 4 possible

        // Take one point and do the projection:
        Vec2d firstp = points1[0];
        Vec2d secondp = points2[0];

        if (!selectGoodSolution(points1[0], points2[0], essentials[i], cameraMatrix[i], cameraMatrix[i+1], R, t)) {
            exit(EXIT_FAILURE);
        }

        relative_transformation[i][0] = R;
        relative_transformation[i][1] = t;

        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl << endl << endl;


    }

    // Defining global coordinates system and camera matrices

    Mat finalCameraMatrices[nr_of_camera][2];
    Mat finalProjectionMatrices[nr_of_camera];


    finalCameraMatrices[0][0] = Mat::eye(3, 3, CV_64F);
    finalCameraMatrices[0][1] = Mat::zeros(3, 1, CV_64F);
    hconcat(finalCameraMatrices[0][0], finalCameraMatrices[0][1], finalProjectionMatrices[0]);
    finalProjectionMatrices[0] = cameraMatrix[0] * finalProjectionMatrices[0];

    for (int i = 1; i < nr_of_camera; ++i) {
        finalCameraMatrices[i][0] = finalCameraMatrices[i - 1][0] * relative_transformation[i - 1][0];
        finalCameraMatrices[i][1] =  finalCameraMatrices[i - 1][0] * relative_transformation[i - 1][1] + finalCameraMatrices[i - 1][1];
        hconcat(finalCameraMatrices[i][0], finalCameraMatrices[i][1], finalProjectionMatrices[i]);
        finalProjectionMatrices[i] = cameraMatrix[i] * finalProjectionMatrices[i];
    }

    cout << endl << endl << "FINAL RESULT " << endl << endl;

    for (int i = 0; i < nr_of_camera; ++i) {
        cout << endl << endl;
        cout << finalCameraMatrices[i][0] << endl;
        cout << finalCameraMatrices[i][1] << endl;

    }

    cout << "outputing for bundle adjustement processing" << endl;
    
    vector<Point3d> points_cloud;
    vector<Vec3b> points_cloud_rgb;
    ofstream file("out.txt");
    if (file.is_open()) {
        // first line:
        // [number of cameras] [number of points] [number of observation]
        file << nr_of_camera << " " << final_points.size() << " " << number_of_observation << endl;

        // [id of camera] [id of point] [x] [y]
        int j = 0;
        for (auto it : final_points) {
            for (auto vec : it.second) {
                file << vec.camera << " " << j << " " << vec.coord.coord_norm.x << " " << vec.coord.coord_norm.y << endl;
            }
            ++j;
        }

        // R,t,f,k1 and k2
        for (int k = 0; k < nr_of_camera; ++k) {
            // R
            Mat Rrod;
            Rodrigues(finalCameraMatrices[k][0], Rrod);
            file << Rrod.at<double>(0, 0) << endl;
            file << Rrod.at<double>(0, 1) << endl;
            file << Rrod.at<double>(0, 2) << endl;

            // t
            file << finalCameraMatrices[k][1].at<double>(0, 0) << endl;
            file << finalCameraMatrices[k][1].at<double>(0, 1) << endl;
            file << finalCameraMatrices[k][1].at<double>(0, 2) << endl;

            // f 
            file << cameraMatrix[k].at<double>(0, 0) << endl;

            // k1
            file << cameraMatrix[k].at<double>(0, 0) << endl;
            // k2 
            file << cameraMatrix[k].at<double>(0, 1) << endl;
        }

        // Now we get an initial projection for all the point using our projection  and the triangulation
        int color = 765;
        for (auto it : final_points) {
            
            Mat pnts3D(1, 1, CV_64FC4);
            vector<Point2f> tmp1, tmp2;
            Observation o1 = it.second.back();
            it.second.pop_back();
            Observation o2 = it.second.back();
            Mat proj1 = finalProjectionMatrices[o1.camera];
            Mat proj2 = finalProjectionMatrices[o2.camera];

            //Vec3d triangPoint = triangulatePoints(Vec2d(o1.coord.coord_real.x, o1.coord.coord_real.y), Vec2d(o2.coord.coord_real.x, o2.coord.coord_real.y), proj1, proj2);
            Mat_<double> triangPoint = IterativeLinearLSTriangulation(Point3d(o1.coord.coord_norm.x, o1.coord.coord_norm.y, 1), proj1, Point3d(o2.coord.coord_norm.x, o2.coord.coord_norm.y, 1), proj2);

            /*file << triangPoint[0] << endl;
            file << triangPoint[1] << endl;
            file << triangPoint[2] << endl;*/
            //points_cloud.push_back(Point3d(triangPoint[0], triangPoint[1], triangPoint[2]));
            points_cloud.push_back(Point3d(triangPoint.at<double>(0,0), triangPoint.at<double>(0,1), triangPoint.at<double>(0,2)));
            points_cloud_rgb.push_back(Vec3b(max(0, (color - 510) % 256), max(0, (color - 255) % 256) , color % 256));
            color -= 1;
        }
    }

    for (int i = 0; i < nr_of_camera; ++i) {
        Matx33d R(finalCameraMatrices[i][0]);
        Vec3d t(finalCameraMatrices[i][1]);
        visualizerShowCamera(R, t, 255.0, 0.0, 0.0, 0.2);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    RunVisualization(points_cloud, points_cloud_rgb, vector<Point3d>(), vector<Vec3b>());
    
    return 0;
}