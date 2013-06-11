/*
Copyright (C) 2013 Giroux Arthur (arthur.giroux@epfl.ch)

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

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
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace std;
using namespace cv;


// Structure to hold points coordinates
typedef struct PointCoord {
    Point2d coord_real; // Real x and y
    Point2d coord_norm; // Normalized x and y
} PointCoord;

// Represent the observation of a point from a specific camera
typedef struct Observation {
    int camera; // Camera index
    PointCoord coord; // Point coordinates
} Observation;



/*
    Triangulate a point given the 2D location from two differents view and two projection matrix
*/
Vec3d triangulatePoints(const Vec2d point1, const Vec2d point2, CvMat proj1, CvMat proj2) {
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

/*
    Convert a Ceres camera to OpenCV matrices R and t
*/
void cameraCeresToOpenCV(double R1, double R2, double R3, double t1, double t2, double t3, Mat& R, Mat& t) {
    Mat Rtmp = (Mat_<double>(3, 1) << R1, R2, R3);
    Rodrigues(Rtmp, R);
    t = (Mat_<double>(3, 1) << t1, t2, t3);
}

/*
    Decompose the Essential Matrix into a Rotation matrix and a Translation matrix using SVD
    see : http://en.wikipedia.org/wiki/Essential_matrix#Determining_R_and_t_from_E
*/
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

    /* tx : tx = V *  Z * V^T
    Mat tx_tmp = V * Z * V_t;

    Mat tx = Mat(3, 1, CV_64F);
    tx.at<double>(0, 0) = tx_tmp.at<double>(2, 1);
    tx.at<double>(1, 0) = tx_tmp.at<double>(0, 2);
    tx.at<double>(2, 0) = tx_tmp.at<double>(1, 0);

    */

    // R = U * W^T * V^T

    R = U * W_t * V_t;

    // See Mastering OpenCV with Practical Computer Vision Projects; Chap. 4
    t = U.col(2);
}


/*
    Return if wheter or not a 3D point lies in front of the two cameras given their projection matrices
*/
bool visibleInBoth(const Point3d point, const Mat proj1, const Mat proj2) {

        // We copy the projection matrix into a 4x4 Matrix
        Mat P4x4 = Mat::eye(4, 4, CV_64F);
        for (int u = 0; u < 3; ++u) {
            for (int v = 0; v < 4; ++v) {
                P4x4.at<double>(u, v) = proj1.at<double>(u, v);
            }
        }
        
        vector<Point3d> in;
        vector<Point3d> out;
        in.push_back(point);
        // We do a perspective transformation onto the first camera
        perspectiveTransform(in, out, P4x4);
        Point3d p1 = out.back();

        out.clear();

        P4x4 = Mat::eye(4, 4, CV_64F);
        for (int u = 0; u < 3; ++u) {
            for (int v = 0; v < 4; ++v) {
                P4x4.at<double>(u, v) = proj2.at<double>(u, v);
            }
        }

        // We do a perspective transformation onto the second camera
        perspectiveTransform(in, out, P4x4);
        Point3d p2 = out.back();
        // We check that both projected points lie in front of the cameras
        return (p1.z > 0 && p2.z > 0);

}

/*
    Check if the given R and t are a valid solution
    It triangulates the normalized point1 and point2 and uses the 3D coordinates to check
    if it's visible in both camera
*/
bool checkGoodSolution(const Point2d point1, const Point2d point2, const Mat R, const Mat t) {
    Mat projMat1, projMat2;
    // We set the first projection matrix to the identity Rotation and no translation
    hconcat(Mat::eye(3, 3, CV_64F), Mat::zeros(3, 1, CV_64F), projMat1);
    // We set the second projection matrix to R and t
    hconcat(R, t, projMat2);

    //Mat_<double> triangPoint = IterativeLinearLSTriangulation(Point3d(point1.x, point1.y, 1), projMat1, Point3d(point2.x, point2.y, 1), projMat2);

    Vec3d triangPoint = triangulatePoints(point1, point2, (CvMat) projMat1, (CvMat) projMat2);

    //return (visibleInBoth(Point3d(testp.at<double>(0, 0), testp.at<double>(0, 1), testp.at<double>(0, 2)), projMat1, projMat2));
    return (visibleInBoth(triangPoint, projMat1, projMat2));
}

/*
    Return the right solution among the 4 possible for R and t
    see: http://en.wikipedia.org/wiki/Essential_matrix#Finding_all_solutions

    return false if no solutions are possible
*/
bool selectGoodSolution(const Point2d point1, const Point2d point2, const Mat E, Mat& R, Mat& t) {

    getRandTfromE(E, R, t);
    if (determinant(R) < 0) {
         getRandTfromE(-E, R, t);
    }

    if (!checkGoodSolution(point1, point2, R, t)) {
        if (!checkGoodSolution(point1, point2, R, -t)) {

            getRandTfromE(E, R, t, true);
            if (determinant(R) < 0) {
                 getRandTfromE(-E, R, t, true);
            }

            if (!checkGoodSolution(point1, point2, R, t)) {
                if (!checkGoodSolution(point1, point2, R, -t)) {
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

/*
    Class to represent the Bundle Adjustement Problem for Ceres Solver
    see: http://homes.cs.washington.edu/~sagarwal/ceres-solver/tutorial.html#bundle-adjustment
    and: https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
*/
class BALProblem {
    public:
        ~BALProblem() {
          delete[] point_index_;
           delete[] camera_index_;
           delete[] observations_;
           delete[] parameters_;
        }

        int num_cameras()            const { return num_cameras_;                    }
        int num_observations()       const { return num_observations_;               }
        const double* observations() const { return observations_;                   }
        double* mutable_cameras()          { return parameters_;                     }
        double* mutable_points()           { return parameters_  + 9 * num_cameras_; }

        double* mutable_camera_for_observation(int i) {
          return mutable_cameras() + camera_index_[i] * 9;
        }
        double* mutable_point_for_observation(int i) {
          return mutable_points() + point_index_[i] * 3;
        }

        BALProblem(int n_camera, int n_points, int n_observations, const Mat cameraMatrices[][2], const Mat intrinsicMatrices[],
            const Mat projectionMatrices[], const Mat distCoeffs[], map<int, vector<Observation> > observations) {

            num_cameras_ = n_camera;
            num_points_ = n_points;
            num_observations_ = n_observations;

            point_index_ = new int[num_observations_];
            camera_index_ = new int[num_observations_];
            observations_ = new double[2 * num_observations_];

            num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
            parameters_ = new double[num_parameters_];

            // Get the cameras parameters
            for (int i = 0; i < num_cameras_; ++i) {
                // R
                Mat Rrod;
                Rodrigues(cameraMatrices[i][0], Rrod);
                parameters_[i*9] = Rrod.at<double>(0, 0);
                parameters_[i*9 + 1] = Rrod.at<double>(0, 1);
                parameters_[i*9 + 2] = Rrod.at<double>(0, 2);

                // t
                parameters_[i*9 + 3] = cameraMatrices[i][1].at<double>(0, 0);
                parameters_[i*9 + 4] = cameraMatrices[i][1].at<double>(0, 1);
                parameters_[i*9 + 5] = cameraMatrices[i][1].at<double>(0, 2);

                // f 
                parameters_[i*9 + 6] = intrinsicMatrices[i].at<double>(0, 0);

                // k1
                parameters_[i*9 + 7] = distCoeffs[i].at<double>(0, 0);
                // k2 
                parameters_[i*9 + 8] = distCoeffs[i].at<double>(0, 1);
            }

            // Get the observations and the 3D initial points
            int k = 0;
            int j = 0;
            for(map<int, vector<Observation> >::iterator it = observations.begin(); it != observations.end(); ++it) {
                // Get the observations
                for(vector<Observation>::iterator item = it->second.begin(); item != it->second.end(); ++item) {
                    camera_index_[j] = item->camera;
                    point_index_[j] = k;
                    observations_[2*j] = item->coord.coord_norm.x;
                    observations_[2*j + 1] = item->coord.coord_norm.y;
                    ++j;
                }

                // We get the first two observations and the associated projection matrices
                Observation o1 = it->second[0];
                Observation o2 = it->second[1];
                Mat proj1 = projectionMatrices[o1.camera];
                Mat proj2 = projectionMatrices[o2.camera];

                // We triangulated using these points
                Mat_<double> triangPoint = IterativeLinearLSTriangulation(Point3d(o1.coord.coord_real.x, o1.coord.coord_real.y, 1), proj1, Point3d(o2.coord.coord_real.x, o2.coord.coord_real.y, 1), proj2);

                parameters_[9*num_cameras_ + 3*k] = triangPoint.at<double>(0,0);
                parameters_[9*num_cameras_ + 3*k + 1] = triangPoint.at<double>(0,1);
                parameters_[9*num_cameras_ + 3*k + 2] = triangPoint.at<double>(0,2);
                ++k;
            }
      }


    private:
        int num_cameras_;
        int num_points_;
        int num_observations_;
        int num_parameters_;

        int* point_index_;
        int* camera_index_;
        double* observations_;
        double* parameters_;
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = - p[0] / p[2];
    T yp = - p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    const T& focal = camera[6];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};


int main(int argc, char** argv) {
    if (argc < 4) {
        cout << "usage: " << argv[0] << " <number_of_camera> <dir_with_intrisic_param> <dir_with_tracking_result>" << endl;
        return -1;
    }

    int nr_of_camera = atoi(argv[1]);
    string dir_intrinsic = argv[2];
    string dir_tracking = argv[3];

    // keep the intrinsic camera matrix
    Mat cameraMatrix[nr_of_camera];
    // keep the distortion coefficients matrix
    Mat distCoeffs[nr_of_camera];

    // The key is the frame number and the value is a vector of observation (camera id; x; y)
    map<int, vector<Observation> > final_points;
    int number_of_observation = 0;

    // 
    map<int, PointCoord> points_red[nr_of_camera];
    map<int, PointCoord> points_green[nr_of_camera];


    // For each camera we get the intrinsic parameters and the tracked points
    for (int i = 0; i < nr_of_camera; ++i) {
        ostringstream oss;
        oss << dir_intrinsic << "/out_camera_data_" << i << ".xml";
        FileStorage fs(oss.str(), FileStorage::READ);
        if (fs.isOpened()) {
            fs["Camera_Matrix"] >> cameraMatrix[i];
            fs["Distortion_Coefficients"] >> distCoeffs[i];
            cout << "camera matrix: " << cameraMatrix[i] << endl << "distortion coeffs: " << distCoeffs[i] << endl;
        }
        fs.release();

        oss.str("");
        oss << dir_tracking << "/cam_" << i << "_red.txt";
        ifstream file;
        file.open(oss.str().c_str());
        if (file.is_open())
        {
            while (file.good())
            { 
                int frame, x, y;
                float radius;
                file >> frame >> x >> y >> radius;
                vector<Point2d> in;
                vector<Point2d> out;
                in.push_back(Point2d(x, y));
                // We undistort and normalize the point
                undistortPoints(in, out, cameraMatrix[i], distCoeffs[i]);
                Point2d pundistort = out.back();
                PointCoord st = { Point2d(x, y), Point2d(pundistort.x, pundistort.y) };
                points_red[i][frame] = st;
            }
            cout << points_red[i].size() << " red points loaded for camera " << i << endl;
            file.close();
        }

        oss.str("");
        oss << dir_tracking << "/cam_" << i << "_green.txt";
        file.open(oss.str().c_str());
        if (file.is_open())
        {
            while (file.good())
            { 
                int frame, x, y;
                float radius;
                file >> frame >> x >> y >> radius;
                vector<Point2d> in;
                vector<Point2d> out;
                in.push_back(Point2d(x, y));
                // We undistort and normalize the point
                undistortPoints(in, out, cameraMatrix[i], distCoeffs[i]);
                Point2d pundistort = out.back();
                PointCoord st = { Point2d(x, y), Point2d(pundistort.x, pundistort.y) };
                points_green[i][frame] = st;
            }
            cout << points_green[i].size() << " green points loaded for camera " << i << endl;
            file.close();
        }

    }

    // Compute pairwise fundamental matrices

    cout << endl << endl << "############# COMPUTING FUNDAMENTAL MATRICES #############" << endl;

    Mat fundamental_matrix;
    Mat essential_matrix;

    // Array of the relatives R and t. [][0] = R and [][1] = t
    Mat relative_transformation[nr_of_camera - 1][2];

    for (int i = 0; i < nr_of_camera - 1; ++i) {
        vector<Point2d> points1;
        vector<Point2d> points2;
        vector<Point2d> points1_norm;
        vector<Point2d> points2_norm;
        int commun = 0;
        for(map<int, PointCoord>::iterator it = points_red[i].begin(); it != points_red[i].end(); ++it) {
            // If the next camera has an observation in the same frame
            if (points_red[i+1].count(it->first) > 0) {
                commun++;
                points1.push_back(it->second.coord_real);
                points2.push_back(points_red[i + 1][it->first].coord_real);
                points1_norm.push_back(it->second.coord_norm);
                points2_norm.push_back(points_red[i + 1][it->first].coord_norm);
                Observation o = {i, it->second};
                final_points[it->first].push_back(o);
                Observation o2 = {i+1, points_red[i + 1][it->first]};
                final_points[it->first].push_back(o2);
                number_of_observation+=2;
            }
        }

        vector<uchar> status;
        // We compute the fundamental matrix
        fundamental_matrix = findFundamentalMat(points1, points2, status);
        cout << "points correspondances : " << commun << endl;
        cout << "points kept : " << countNonZero(status) << endl;
        cout << "matrix found : " << endl << fundamental_matrix << endl;

        // getting the essential matrix:
        // E = K'^T * F * K

        essential_matrix = cameraMatrix[i+1].t() * fundamental_matrix * cameraMatrix[i];

        // We check a requirement for the essential matrix
        if (fabsf(determinant(essential_matrix)) - 1.0 > 1e-07) {
            cerr << "det(E) != 0 : " << determinant(essential_matrix) << endl;
            exit(EXIT_FAILURE);
        }

        cout << "ESSENTIAL MATRIX " << endl;
        cout << essential_matrix << endl;

        Mat R(3, 3, CV_64F);
        Mat t(3, 1, CV_64F);

        // Find the right solution among the 4 possible

        // Take one point and do the projection:
        Vec2d firstp;
        Vec2d secondp;
        bool foundsolution = false;
        for (int k = 0; k < status.size(); k++) {
            if (status[k] == 1) {
                cout << "taking point " << k << endl;
                firstp = points1_norm[k];
                secondp = points2_norm[k];
                if (selectGoodSolution(firstp, secondp, essential_matrix, R, t)) {
                    foundsolution = true;
                    break;
                }
            }
        }

        if (!foundsolution) {
            cerr << "something went wrong, couldn't find a good solution among the 4 possible during the decomposition of E" << endl;
            exit(EXIT_FAILURE);
        }

        // We set the relative transformation to the right solution
        relative_transformation[i][0] = R;
        relative_transformation[i][1] = t;

        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl << endl << endl;


    }

    // Defining global coordinates system and camera matrices

    // final camera matrix; [][0] = R, [][1] = t
    Mat finalCameraMatrices[nr_of_camera][2];

    // Projection matrix = K * P
    Mat finalProjectionMatrices[nr_of_camera];

    // We set the first camera matrix to identity rotation and zero translation
    finalCameraMatrices[0][0] = Mat::eye(3, 3, CV_64F);
    finalCameraMatrices[0][1] = Mat::zeros(3, 1, CV_64F);
    hconcat(finalCameraMatrices[0][0], finalCameraMatrices[0][1], finalProjectionMatrices[0]);
    finalProjectionMatrices[0] = cameraMatrix[0] * finalProjectionMatrices[0];

    // Then for the other cameras we express them relative to the first one
    for (int i = 1; i < nr_of_camera; ++i) {
        finalCameraMatrices[i][0] = finalCameraMatrices[i - 1][0] * relative_transformation[i - 1][0];
        finalCameraMatrices[i][1] =  finalCameraMatrices[i - 1][0] * relative_transformation[i - 1][1] + finalCameraMatrices[i - 1][1];
        hconcat(finalCameraMatrices[i][0], finalCameraMatrices[i][1], finalProjectionMatrices[i]);
        finalProjectionMatrices[i] = cameraMatrix[i] * finalProjectionMatrices[i];
    }

    cout << endl << endl << "FINAL CAMERA MATRICES " << endl << endl;

    for (int i = 0; i < nr_of_camera; ++i) {
        cout << endl << endl;
        cout << finalCameraMatrices[i][0] << endl;
        cout << finalCameraMatrices[i][1] << endl;

    }

    // We create an instance of the Bundle Adjustement Problem
    BALProblem bal_problem(nr_of_camera, final_points.size(), number_of_observation, finalCameraMatrices, cameraMatrix,
        finalProjectionMatrices, distCoeffs, final_points);


    const double* observations = bal_problem.observations();

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    for (int i = 0; i < bal_problem.num_observations(); ++i) {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.

        ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0],
           observations[2 * i + 1]);
        problem.AddResidualBlock(cost_function,
                             NULL /* squared loss */,
           bal_problem.mutable_camera_for_observation(i),
           bal_problem.mutable_point_for_observation(i));
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";


    // We show the original camera matrix in red

    for (int k = 0; k < nr_of_camera; ++k) {
        visualizerShowCamera(finalCameraMatrices[k][0], finalCameraMatrices[k][1], 0.0, 0.0, 255.0, 0.2);
    }

    // We show the new camera matrix after bundle adjustment in blue
    double* cam = bal_problem.mutable_cameras();

    for (int k = 0; k < bal_problem.num_cameras(); ++k) {
        Mat R, t;
        cameraCeresToOpenCV(cam[k*9], cam[k*9 + 1], cam[k*9 + 2], cam[k*9 + 3], cam[k*9 + 4], cam[k*9 + 5], R, t);
        visualizerShowCamera(R, t, 255.0, 0.0, 0.0, 0.2);
    }

    // We show the triangulated 3D points
    vector<Point3d> points_cloud;
    vector<Vec3b> points_cloud_rgb;

    for(map<int, vector<Observation> >::iterator it = final_points.begin(); it != final_points.end(); ++it) {
        Mat pnts3D(1, 1, CV_64FC4);
        vector<Point2f> tmp1, tmp2;
        Observation o1 = it->second[it->second.size() - 2];
        Observation o2 = it->second[it->second.size() - 1];
        Mat proj1 = finalProjectionMatrices[o1.camera];
        Mat proj2 = finalProjectionMatrices[o2.camera];

        //Vec3d triangPoint = triangulatePoints(Vec2d(o1.coord.coord_real.x, o1.coord.coord_real.y), Vec2d(o2.coord.coord_real.x, o2.coord.coord_real.y), proj1, proj2);
        Mat_<double> triangPoint = IterativeLinearLSTriangulation(Point3d(o1.coord.coord_real.x, o1.coord.coord_real.y, 1), proj1, Point3d(o2.coord.coord_real.x, o2.coord.coord_real.y, 1), proj2);
        //points_cloud.push_back(Point3d(triangPoint[0], triangPoint[1], triangPoint[2]));
        points_cloud.push_back(Point3d(triangPoint.at<double>(0,0), triangPoint.at<double>(0,1), triangPoint.at<double>(0,2)));
        points_cloud_rgb.push_back(Vec3b(255, 255, 255));
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    RunVisualization(points_cloud, points_cloud_rgb, vector<Point3d>(), vector<Vec3b>());

    return 0;
}