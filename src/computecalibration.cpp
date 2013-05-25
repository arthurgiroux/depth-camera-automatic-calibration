#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

typedef struct Observation {
    int camera;
    double x;
    double y;
} Observation;

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

    map<int, Vec3f> points_red[nr_of_camera];
    map<int, Vec3f> points_green[nr_of_camera];

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
                vector<Point2f> tmp;
                vector<Point2f> out;
                tmp.push_back(Point2f(x, y));
                undistortPoints(tmp, out, cameraMatrix[i], distCoeffs[i]);
                Point2f pundistort = out.back();
                points_red[i][frame] = Vec3f(pundistort.x, pundistort.y, radius);
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
                points_green[i][frame] = Vec3f(x, y, radius);
            }
            cout << points_green[i].size() << " green points loaded for camera " << i << endl;
            file.close();
        }

    }

    // Compute pairwise fundamental matrices

    Mat fundamentals[nr_of_camera - 1];
    Mat essentials[nr_of_camera - 1];

    Mat relative_transformation[nr_of_camera - 1][2];

    for (int i = 0; i < nr_of_camera - 1; ++i) {
        vector<Vec2f> points1;
        vector<Vec2f> points2;
        int commun = 0;

        for (map<int, Vec3f>::iterator it = points_red[i].begin(); it != points_red[i].end(); ++it) {
            if (points_red[i+1].count(it->first) > 0) {
                commun++;
                points1.push_back(Vec2f(it->second[0], it->second[1]));
                points2.push_back(Vec2f(points_red[i + 1][it->first][0], points_red[i + 1][it->first][1]));
                final_points[it->first].push_back({i, it->second[0], it->second[1]});
                final_points[it->first].push_back({i+1, points_red[i + 1][it->first][0], points_red[i + 1][it->first][1]});
                number_of_observation++;
            }
        }

        cout << "commun " << commun << endl;
        fundamentals[i] = findFundamentalMat(points1, points2);
        cout << fundamentals[i] << endl;


        for (unsigned int j = 0; j < points1.size(); j++) {
            Mat p = Mat(Vec3f(points1[j][0], points1[j][1], 1));
            Mat p_2 = Mat(Vec3f(points2[j][0], points2[j][1], 1));
            p.convertTo(p, CV_64F);
            p_2.convertTo(p_2, CV_64F);
            Mat check = p.t() * fundamentals[i] * p_2;

            if (check.at<double>(0, 0)  <= -1 || check.at<double>(0, 0)  >= 1) {
                //cout << "PROBLEM DURING PAIRWISE TRANSFORMATION WITH CAMERAS " << i << " ";
                //cout << (i+1) << " not close to 0 : " << check.at<double>(0, 0) << endl;
            }
        }

        // getting the essential matrix:
        // E = K'^T * F * K

        essentials[i] = cameraMatrix[i].t() * fundamentals[i] * cameraMatrix[i+1];
        cout << "ESSENTIAL MATRIX " << endl;
        cout << essentials[i] << endl;

        // SVD DECOMPOSITON

        SVD svd = SVD(essentials[i]);
        Mat U = svd.u;
        //Mat S_vector = svd.w;
        Mat V_t = svd.vt;
        Mat V = V_t.t();
        //cout << S_vector << endl;

        /*double S_data[3][3] = { {S_vector.at<double>(0), 0, 0}, 
                                {0,  S_vector.at<double>(1), 0}, 
                                {0,  0, S_vector.at<double>(2)} };

        Mat S = Mat(3, 3, CV_64F, &S_data);*/

        double W_t_data[3][3] = { {0, 1, 0},
                                {-1,  0, 0},
                                {0,  0, 1} };

        Mat W_t = Mat(3, 3, CV_64F, &W_t_data);

        double Z_data[3][3] = { {0, -1, 0},
                                {1,  0, 0},
                                {0,  0, 0} };

        Mat Z = Mat(3, 3, CV_64F, &Z_data);


        // OLD : tx = V * W * S * V^T
        //Mat tx = V * W * S * V_t;

        // NEW : tx = V *  Z * V^T
        Mat tx_tmp = V * Z * V_t;

        Mat tx = Mat(3, 1, CV_64F);
        tx.at<double>(0, 0) = tx_tmp.at<double>(2, 1);
        tx.at<double>(1, 0) = tx_tmp.at<double>(0, 2);
        tx.at<double>(2, 0) = tx_tmp.at<double>(1, 0);

        cout << "translation : " << endl;
        cout << tx << endl;

        // R = U * W^T * V^T

        Mat R = U * W_t * V_t;

        cout << "rotation : " << endl;
        cout << R << endl;

        relative_transformation[i][0] = R;
        relative_transformation[i][1] = tx;

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
        cout << finalProjectionMatrices[i] << endl << endl;

    }

    cout << "outputing for bundle adjustement processing" << endl;

    ofstream file("out.txt");
    if (file.is_open()) {
        // first line:
        // [number of cameras] [number of points] [number of observation]
        file << nr_of_camera << " " << final_points.size() << " " << number_of_observation << endl;

        // [id of camera] [id of point] [x] [y]
        int j = 0;
        for (auto it : final_points) {
            for (auto vec : it.second) {
                file << vec.camera << " " << j << " " << vec.x << " " << vec.y << endl;
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
        for (auto it : final_points) {
            
            Mat pnts3D(1, 1, CV_64FC4);
            vector<Point2f> tmp1, tmp2;
            Observation o1 = it.second.back();
            it.second.pop_back();
            Observation o2 = it.second.back();
            tmp1.push_back(Point2f(o1.x, o1.y));
            tmp2.push_back(Point2f(o2.x, o2.y));
            CvMat proj1 = finalProjectionMatrices[o1.camera];
            CvMat proj2 = finalProjectionMatrices[o2.camera];

            //cvTriangulatePoints(&proj1, &proj2, tmp1, tmp2, pnts3D);
            //cout << pnts3D << endl;

            CvMat *p1 = cvCreateMat(2, 1, CV_64FC1);
            CvMat *p2 = cvCreateMat(2, 1, CV_64FC1);
            CV_MAT_ELEM( *p1, double, 0, 0 ) = o1.x;
            CV_MAT_ELEM( *p1, double, 1, 0 ) = o1.y;
            CV_MAT_ELEM( *p2, double, 0, 0 ) = o2.x;
            CV_MAT_ELEM( *p2, double, 1, 0 ) = o2.y;

            //triangulate both projections to find real point position
            //!!! all parameters must be double type
            CvMat *point3D = cvCreateMat(4, 1, CV_64F);
            //cout << Mat(&proj1) << endl;
            //cout << Mat(&proj2) << endl;

            cvTriangulatePoints(&proj1, &proj2, p1, p2, point3D);

            //to get the real position we need to do also a homogeneous division
            point3D->data.db[0] /= point3D->data.db[3];
            point3D->data.db[1] /= point3D->data.db[3];
            point3D->data.db[2] /= point3D->data.db[3];
            file << point3D->data.db[0] << endl;
            file << point3D->data.db[1] << endl;
            file << point3D->data.db[2] << endl;
        }
    }
    
    return 0;
}