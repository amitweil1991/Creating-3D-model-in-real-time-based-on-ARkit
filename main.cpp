
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Frame.h"
#include "grid3D.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "include/include/dualmc.h"
#include <ostream>
#include <fstream>
#include "marchingCubes.h"
#define FEATURES_NUM_TO_PRESENT 1
#define STARTING_FEATURE_TO_PRESENT 0
//using namespace cv;
//using namespace std;
using json = nlohmann::json;

static const int N = 30;

void video ();
void testProjection(vector<worldPoint> image1_first_features, vector<string> imagesPath,vector<Frame> frames, int* cntr);
Matx31f calc_avg_grid_center(vector<worldPoint> features_point);


/** function for presenting 25 rays from different frames (each vector in features is a vector of features from a different frame)
 *
 * @param cam_poses - the cam poses in the GRID axis, together with the features it creates the rays
 * @param frames_featurs - the features that creates the rays
 * @param grid - the given grid we're presenting the rays on
 */
void CreateRaysAndVoxels(vector<Matx31f>& cam_poses, vector<vector<worldPoint>>& frames_featurs, grid3D& grid,
        int number_of_frames_to_present) {
    vector<vector<tuple<int, int, int>>> keys;
    for (int i = 0; i < number_of_frames_to_present; i++) {
//        cout << "frame num " << i << endl;
        for (int j = STARTING_FEATURE_TO_PRESENT; j <  frames_featurs[i].size(); ++j) {
//            cout << "frame num " << i << " " << "feature_num " << j << endl;
            Matx31f point_in_world = convertWorldPointToMatx(frames_featurs[i][j]);
            Matx31f point_in_grid = grid.mapFromWorldToGrid(point_in_world);
            straight_line_equation l(cam_poses[i-1], point_in_grid);
//            cout << "CAMPOSE: " <<  cam_poses[i-1] << endl;
//            cout << "FEATURE_IN_GRID: " << point_in_grid << endl;
            Point2d slope = grid.getSlopeRange(l);
            if (slope.x == -9999 && slope.y == -9999) {
                continue;
            }
            tuple<Matx31f, Matx31f> intersection_points = grid.findIntersectionPoint(l, slope);
            Matx31f entrance_point = get<0>(intersection_points);
            keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point(0, 0), entrance_point(1, 0),entrance_point(2, 0),l,i));
            if (keys.back().empty()) {
                continue;
            }
            grid.bresenhamAlgorithim(l, keys.back()[0], i);

//            cout << "iteration " << i << "Range of S: " << "(" << slope.x << " , " << slope.y << ")" << endl;



            }
        }
//    cout << " **************************************************************************************" << endl;
//    for (auto it = grid.getGrid().begin(); it != grid.getGrid().end(); it++) {
//        tuple<int, int, int> key = it->first;
//        cout << get<0>(key) << ", " << get<1>(key) << ", " << get<2>(key) << endl;
//        if(it->second.getDistance() > 0 ) {
//            cout << "DISTANCEY: " << it->second.getDistance() << endl << endl;
//        }
//        else{
//            cout << "NEGATIVE" << endl;
//        }
//
//
//    }
}


void testForChackTheDistance(grid3D& grid){
    vector<vector<tuple<int, int, int>>> keys;

    Matx31f grid_center(0,0,0);
    Matx31f dummy_campos(-202,0,0);

//    Matx31f grid_center2(-2,0,0);
//    Matx31f dummy_campos2(-2,-6,0);

//    Matx31f grid_center3(2,2,0);
//    Matx31f dummy_campos3(6,2,0);
//
//    Matx31f grid_center4(2,0,0);
//    Matx31f dummy_campos4(2,6,0);

//    Matx31f grid_center5(5,-5,-5);
//    Matx31f dummy_campos5(6,-5,-5);
//
//    Matx31f grid_center6(5,-5,5);
//    Matx31f dummy_campos6(6,-5,5);
//
//    Matx31f grid_center7(5,5,-5);
//    Matx31f dummy_campos7(6,5,-5);
//
//    Matx31f grid_center8(5,5,5);
//    Matx31f dummy_campos8(6,5,5);


    straight_line_equation l(dummy_campos, grid_center);
//    straight_line_equation l2(dummy_campos2, grid_center2);
//    straight_line_equation l3(dummy_campos3, grid_center3);
//    straight_line_equation l4(dummy_campos4, grid_center4);
//    straight_line_equation l5(dummy_campos5, grid_center5);
//    straight_line_equation l6(dummy_campos6, grid_center6);
//    straight_line_equation l7(dummy_campos7, grid_center7);
//    straight_line_equation l8(dummy_campos8, grid_center8);

    Point2d slope = grid.getSlopeRange(l);
//    Point2d slope2 = grid.getSlopeRange(l2);
//    Point2d slope3 = grid.getSlopeRange(l3);
//    Point2d slope4 = grid.getSlopeRange(l4);
//    Point2d slope5 = grid.getSlopeRange(l5);
//    Point2d slope6 = grid.getSlopeRange(l6);
//    Point2d slope7 = grid.getSlopeRange(l7);
//    Point2d slope8 = grid.getSlopeRange(l8);

    tuple<Matx31f, Matx31f> intersection_points = grid.findIntersectionPoint(l, slope);
    Matx31f entrance_point = get<0>(intersection_points);
    keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point(0, 0), entrance_point(1, 0),entrance_point(2, 0),l,100));
    grid.bresenhamAlgorithim(l, keys.back()[0], 100);

//    tuple<Matx31f, Matx31f> intersection_points2 = grid.findIntersectionPoint(l2, slope2);
//    Matx31f entrance_point2 = get<0>(intersection_points2);
//    keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point2(0, 0), entrance_point2(1, 0),entrance_point2(2, 0),l2,100));
//    grid.bresenhamAlgorithim(l2, keys.back()[0], 100);

//    tuple<Matx31f, Matx31f> intersection_points3 = grid.findIntersectionPoint(l3, slope3);
//    Matx31f entrance_point3 = get<0>(intersection_points3);
//    keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point3(0, 0), entrance_point3(1, 0),entrance_point3(2, 0),l3,100));
//    grid.bresenhamAlgorithim(l3, keys.back()[0], 100);
//
//    tuple<Matx31f, Matx31f> intersection_points4 = grid.findIntersectionPoint(l4, slope4);
//    Matx31f entrance_point4 = get<0>(intersection_points4);
//    keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point4(0, 0), entrance_point4(1, 0),entrance_point4(2, 0),l4,100));
//    grid.bresenhamAlgorithim(l4, keys.back()[0], 100);
//
//    tuple<Matx31f, Matx31f> intersection_points5 = grid.findIntersectionPoint(l5, slope5);
//    Matx31f entrance_point5 = get<0>(intersection_points5);
//    keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point5(0, 0), entrance_point5(1, 0),entrance_point5(2, 0),l5,100));
//    grid.bresenhamAlgorithim(l5, keys.back()[0], 100);
//
//    tuple<Matx31f, Matx31f> intersection_points6 = grid.findIntersectionPoint(l6, slope6);
//    Matx31f entrance_point6 = get<0>(intersection_points6);
//    keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point6(0, 0), entrance_point6(1, 0),entrance_point6(2, 0),l6,100));
//    grid.bresenhamAlgorithim(l6, keys.back()[0], 100);
//
//    tuple<Matx31f, Matx31f> intersection_points7 = grid.findIntersectionPoint(l7, slope7);
//    Matx31f entrance_point7 = get<0>(intersection_points7);
//    keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point7(0, 0), entrance_point7(1, 0),entrance_point7(2, 0),l7,100));
//    grid.bresenhamAlgorithim(l7, keys.back()[0], 100);
//
//    tuple<Matx31f, Matx31f> intersection_points8 = grid.findIntersectionPoint(l8, slope8);
//    Matx31f entrance_point8 = get<0>(intersection_points8);
//    keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point8(0, 0), entrance_point8(1, 0),entrance_point8(2, 0),l8,100));
//    grid.bresenhamAlgorithim(l8, keys.back()[0], 100);


//    for (auto it = grid.getGrid().begin(); it != grid.getGrid().end(); it++) {
//        tuple<int, int, int> key = it->first;
//        cout << get<0>(key) << ", " << get<1>(key) << ", " << get<2>(key) << endl;
//        cout << "DISTANCE " << it->second.getDistance() << endl << endl;


//    }
}




/*!
 * function for creating a mesh in a shape of a ball, for testing purposes
 * @param grid - the grid that will hold the values of the ball, we will then create a mesh from it.
 */
void createBall(grid3D& grid){
    // fill the grid with distances matching a ball
        int nn = 0;
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                for (int k = 0; k < N; k++) {
                    float r2 = sqrtf((i - N / 2)*(i - N / 2) + (j - N / 2)*(j - N / 2) + (k - N / 2)*(k - N / 2));
                    tuple<int,int,int> tup_i_j_k(i, j , k);
                    voxel new_voxel(r2 - N / 4, 100);
                    pair<tuple<int,int,int>, voxel> new_cell(tup_i_j_k, new_voxel);
                    grid.getDensifiedGrid().insert(new_cell);
                }
            }
        }
        grid.createGridCellMap();
        //polygonize each cell
        std::vector<Mesh> gridMeshes;
        CreateMeshFromGrid(gridMeshes, grid);
        //collect all to single mesh
        Mesh fullmesh;
        createFullMesh(gridMeshes, fullmesh);
        //export to file
        std::ofstream myfile;
        myfile.open("example.off");
        myfile << "OFF\n";
        myfile << fullmesh.v.size() <<" "<<fullmesh.t.size()<<" 0\n";
        for (int i = 0; i < fullmesh.v.size(); i++) {
            myfile << fullmesh.v[i](0) << " " << fullmesh.v[i](1) << " " << fullmesh.v[i](2) << "\n";
        }
        for (int i = 0; i < fullmesh.t.size(); i++) {
            myfile << "3 "<< fullmesh.t[i].I[0] << " " << fullmesh.t[i].I[1] << " " << fullmesh.t[i].I[2] << "\n";
        }

        myfile.close();

    }


/** function for presenting 25 rays from different frames with the same feature, just in order to check that the intersection point
 * between all the rays, which is the feature itslef will have greater confidence
 *
 * @param cam_poses - the cam poses in the GRID axis, together with the features it creates the rays
 * @param frames_featurs - the features that creates the rays
 * @param grid - the given grid we're presenting the rays on
 */
void testPresentTwentyFiveRaysWithSameFeature(vector<Matx31f>& cam_poses, vector<vector<worldPoint>>& frames_featurs, grid3D& grid) {
    vector<vector<tuple<int, int, int>>> keys;
    Matx31f point_in_world = convertWorldPointToMatx(frames_featurs[0][102]);
    Matx31f point_in_grid = grid.mapFromWorldToGrid(point_in_world);
    for(int i = 0; i < cam_poses.size(); i++ ) {
        straight_line_equation l(cam_poses[i], point_in_grid);
        Point2d slope = grid.getSlopeRange(l);
        if (slope.x == -9999 && slope.y == -9999) {
            continue;
        }
        tuple<Matx31f, Matx31f> intersection_points = grid.findIntersectionPoint(l, slope);
        Matx31f entrance_point = get<0>(intersection_points);
        keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point(0, 0), entrance_point(1, 0),
                                                          entrance_point(2, 0), l, i));
        if (keys.back().empty()) {
            continue;
        }
        grid.bresenhamAlgorithim(l, keys.back()[0], i);

//            cout << "iteration " << i << "Range of S: " << "(" << slope.x << " , " << slope.y << ")" << endl;

//        for (auto it = grid.getGrid().begin(); it != grid.getGrid().end(); it++) {
//            tuple<int, int, int> key = it->first;
//            cout << get<0>(key) << ", " << get<1>(key) << ", " << get<2>(key) << endl;
//
//        }
    }

}


/** function for creating a vector with all the camera positions in a given grid axis, from the given frames **/
void createCamPoseVectorForFrames(vector<Matx31f>& cam_pose_vector, vector<Frame>& frames, grid3D& grid ){
    for(int i = 0; i < frames.size(); i++){
        cam_pose_vector.push_back(grid.mapFromWorldToGrid(frames[i].getCampos()));
//        cout << "camPose of frame " << i << " is: ";
//        cout << grid.mapFromWorldToGrid(frames[i].getCampos()) << endl;
    }

}




/*!
 * this function is responsible for parsing all the json file and filling up all the images_path, creating all the frames
 * and parsing all the features in each frame to the features vector
 * @param images_path - will contain all the paths to the images in the local machine
 * @param frames_vector - will contatin all the frames that we've created from within the function
 * @param features_vector - vector of vectors, where each vector is all the features of a given image.
 */
void ParseAllJsons(vector<string>& images_path, vector<Frame>& frames_vector,vector<vector<worldPoint>>& features_vector)

{
    /// Json file that the Grid build from
    vector<float> image1_C1;
    vector<float> image1_C2;
    vector<worldPoint> image1_first_features;
    string image1_s="100000437.json";
    parseJson(image1_s ,&image1_C1 ,&image1_C2 , &image1_first_features);
    string image1_path = "images/100000437.jpg";
    images_path.push_back(image1_path);

    vector<float> image2_C1;
    vector<float> image2_C2;
    vector<worldPoint> image2_first_features;
    string image2_s="100000467.json";
    parseJson(image2_s ,&image2_C1 ,&image2_C2 , &image2_first_features);
    string image2_path = "images/100000467.jpg";
    images_path.push_back(image2_path);


    vector<float> image3_C1;
    vector<float> image3_C2;
    vector<worldPoint> image3_first_features;
    string image3_s="100000497.json";
    parseJson(image3_s ,&image3_C1 ,&image3_C2 , &image3_first_features);
    string image3_path = "images/100000497.jpg";
    images_path.push_back(image3_path);

    vector<float> image4_C1;
    vector<float> image4_C2;
    vector<worldPoint> image4_first_features;
    string image4_s="100000527.json";
    parseJson(image4_s ,&image4_C1 ,&image4_C2 , &image4_first_features);
    string image4_path = "images/100000527.jpg";
    images_path.push_back(image4_path);

    vector<float> image5_C1;
    vector<float> image5_C2;
    vector<worldPoint> image5_first_features;
    string image5_s="100000557.json";
    parseJson(image5_s ,&image5_C1 ,&image5_C2 , &image5_first_features);
    string image5_path = "images/100000557.jpg";
    images_path.push_back(image5_path);

    vector<float> image6_C1;
    vector<float> image6_C2;
    vector<worldPoint> image6_first_features;
    string image6_s="100000587.json";
    parseJson(image6_s ,&image6_C1 ,&image6_C2 , &image6_first_features);
    string image6_path = "images/100000587.jpg";
    images_path.push_back(image6_path);

    vector<float> image7_C1;
    vector<float> image7_C2;
    vector<worldPoint> image7_first_features;
    string image7_s="100000617.json";
    parseJson(image7_s ,&image7_C1 ,&image7_C2 , &image7_first_features);
    string image7_path = "images/100000617.jpg";
    images_path.push_back(image7_path);

    vector<float> image8_C1;
    vector<float> image8_C2;
    vector<worldPoint> image8_first_features;
    string image8_s="100000647.json";
    parseJson(image8_s ,&image8_C1 ,&image8_C2 , &image8_first_features);
    string image8_path = "images/100000647.jpg";
    images_path.push_back(image8_path);

    vector<float> image9_C1;
    vector<float> image9_C2;
    vector<worldPoint> image9_first_features;
    string image9_s="100000677.json";
    parseJson(image9_s ,&image9_C1 ,&image9_C2 , &image9_first_features);
    string image9_path = "images/100000677.jpg";
    images_path.push_back(image9_path);

    vector<float> image10_C1;
    vector<float> image10_C2;
    vector<worldPoint> image10_first_features;
    string image10_s="100000707.json";
    parseJson(image10_s ,&image10_C1 ,&image10_C2 , &image10_first_features);
    string image10_path = "images/100000707.jpg";
    images_path.push_back(image10_path);

    vector<float> image11_C1;
    vector<float> image11_C2;
    vector<worldPoint> image11_first_features;
    string image11_s="100000737.json";
    parseJson(image11_s ,&image11_C1 ,&image11_C2 , &image11_first_features);
    string image11_path = "images/100000737.jpg";
    images_path.push_back(image11_path);

    vector<float> image12_C1;
    vector<float> image12_C2;
    vector<worldPoint> image12_first_features;
    string image12_s="100000767.json";
    parseJson(image12_s ,&image12_C1 ,&image12_C2 , &image12_first_features);
    string image12_path = "images/100000767.jpg";
    images_path.push_back(image12_path);

    vector<float> image13_C1;
    vector<float> image13_C2;
    vector<float> image14_C1;
    vector<float> image14_C2;
    vector<float> image15_C1;
    vector<float> image15_C2;
    vector<float> image16_C1;
    vector<float> image16_C2;
    vector<float> image17_C1;
    vector<float> image17_C2;
    vector<float> image18_C1;
    vector<float> image18_C2;
    vector<float> image19_C1;
    vector<float> image19_C2;
    vector<float> image20_C1;
    vector<float> image20_C2;
    vector<float> image21_C1;
    vector<float> image21_C2;
    vector<float> image22_C1;
    vector<float> image22_C2;
    vector<float> image23_C1;
    vector<float> image23_C2;
    vector<float> image24_C1;
    vector<float> image24_C2;
    vector<float> image25_C1;
    vector<float> image25_C2;
    vector<float> image26_C1;
    vector<float> image26_C2;
    vector<float> image27_C1;
    vector<float> image27_C2;
    vector<float> image28_C1;
    vector<float> image28_C2;
    vector<float> image29_C1;
    vector<float> image29_C2;
    vector<float> image30_C1;
    vector<float> image30_C2;
    vector<float> image31_C1;
    vector<float> image31_C2;
    vector<float> image32_C1;
    vector<float> image32_C2;
    vector<float> image33_C1;
    vector<float> image33_C2;
    vector<float> image34_C1;
    vector<float> image34_C2;
    vector<float> image35_C1;
    vector<float> image35_C2;
    vector<float> image36_C1;
    vector<float> image36_C2;
    vector<float> image37_C1;
    vector<float> image37_C2;
    vector<float> image38_C1;
    vector<float> image38_C2;
    vector<float> image39_C1;
    vector<float> image39_C2;
    vector<float> image40_C1;
    vector<float> image40_C2;
    vector<float> image41_C1;
    vector<float> image41_C2;
    vector<float> image42_C1;
    vector<float> image42_C2;
    vector<float> image43_C1;
    vector<float> image43_C2;
    vector<float> image44_C1;
    vector<float> image44_C2;
    vector<float> image45_C1;
    vector<float> image45_C2;
    vector<float> image46_C1;
    vector<float> image46_C2;
    vector<float> image47_C1;
    vector<float> image47_C2;
    vector<float> image48_C1;
    vector<float> image48_C2;
    vector<float> image49_C1;
    vector<float> image49_C2;
    vector<float> image50_C1;
    vector<float> image50_C2;

    vector<worldPoint> image13_first_features;
    vector<worldPoint> image14_first_features;
    vector<worldPoint> image15_first_features;
    vector<worldPoint> image16_first_features;
    vector<worldPoint> image17_first_features;
    vector<worldPoint> image18_first_features;
    vector<worldPoint> image19_first_features;
    vector<worldPoint> image20_first_features;
    vector<worldPoint> image21_first_features;
    vector<worldPoint> image22_first_features;
    vector<worldPoint> image23_first_features;
    vector<worldPoint> image24_first_features;
    vector<worldPoint> image25_first_features;
    vector<worldPoint> image26_first_features;
    vector<worldPoint> image27_first_features;
    vector<worldPoint> image28_first_features;
    vector<worldPoint> image29_first_features;
    vector<worldPoint> image30_first_features;
    vector<worldPoint> image31_first_features;
    vector<worldPoint> image32_first_features;
    vector<worldPoint> image33_first_features;
    vector<worldPoint> image34_first_features;
    vector<worldPoint> image35_first_features;
    vector<worldPoint> image36_first_features;
    vector<worldPoint> image37_first_features;
    vector<worldPoint> image38_first_features;
    vector<worldPoint> image39_first_features;
    vector<worldPoint> image40_first_features;
    vector<worldPoint> image41_first_features;
    vector<worldPoint> image42_first_features;
    vector<worldPoint> image43_first_features;
    vector<worldPoint> image44_first_features;
    vector<worldPoint> image45_first_features;
    vector<worldPoint> image46_first_features;
    vector<worldPoint> image47_first_features;
    vector<worldPoint> image48_first_features;
    vector<worldPoint> image49_first_features;
    vector<worldPoint> image50_first_features;



    string image13_s="100000797.json";
    string image14_s="100000827.json";
    string image15_s="100000857.json";
    string image16_s="100000887.json";
    string image17_s="100000917.json";
    string image18_s="100000947.json";
    string image19_s="100000977.json";
    string image20_s="100001007.json";
    string image21_s="100001037.json";
    string image22_s="100001067.json";
    string image23_s="100001097.json";
    string image24_s="100001127.json";
    string image25_s="100001157.json";
    string image26_s="100001187.json";
    string image27_s="100001217.json";
    string image28_s="100001247.json";
    string image29_s="100001277.json";
    string image30_s="100001307.json";
    string image31_s="100001337.json";
    string image32_s="100001367.json";
    string image33_s="100001397.json";
    string image34_s="100001427.json";
    string image35_s="100001457.json";
    string image36_s="100001487.json";
    string image37_s="100001517.json";
    string image38_s="100001547.json";
    string image39_s="100001577.json";
    string image40_s="100001607.json";
    string image41_s="100001637.json";
    string image42_s="100001667.json";
    string image43_s="100001697.json";
    string image44_s="100001727.json";
    string image45_s="100001757.json";
    string image46_s="100001787.json";
    string image47_s="100001817.json";
    string image48_s="100001847.json";
    string image49_s="100001877.json";
    string image50_s="100001907.json";

    parseJson(image13_s ,&image13_C1 ,&image13_C2 , &image13_first_features);
    parseJson(image14_s ,&image14_C1 ,&image14_C2 , &image14_first_features);
    parseJson(image15_s ,&image15_C1 ,&image15_C2 , &image15_first_features);
    parseJson(image16_s ,&image16_C1 ,&image16_C2 , &image16_first_features);
    parseJson(image17_s ,&image17_C1 ,&image17_C2 , &image17_first_features);
    parseJson(image18_s ,&image18_C1 ,&image18_C2 , &image18_first_features);
    parseJson(image19_s ,&image19_C1 ,&image19_C2 , &image19_first_features);
    parseJson(image20_s ,&image20_C1 ,&image20_C2 , &image20_first_features);
    parseJson(image21_s ,&image21_C1 ,&image21_C2 , &image21_first_features);
    parseJson(image22_s ,&image22_C1 ,&image22_C2 , &image22_first_features);
    parseJson(image23_s ,&image23_C1 ,&image23_C2 , &image23_first_features);
    parseJson(image24_s ,&image24_C1 ,&image24_C2 , &image24_first_features);
    parseJson(image25_s ,&image25_C1 ,&image25_C2 , &image25_first_features);
    parseJson(image26_s ,&image26_C1 ,&image26_C2 , &image26_first_features);
    parseJson(image27_s ,&image27_C1 ,&image27_C2 , &image27_first_features);
    parseJson(image28_s ,&image28_C1 ,&image28_C2 , &image28_first_features);
    parseJson(image29_s ,&image29_C1 ,&image29_C2 , &image29_first_features);
    parseJson(image30_s ,&image30_C1 ,&image30_C2 , &image30_first_features);
    parseJson(image31_s ,&image31_C1 ,&image31_C2 , &image31_first_features);
    parseJson(image32_s ,&image32_C1 ,&image32_C2 , &image32_first_features);
    parseJson(image33_s ,&image33_C1 ,&image33_C2 , &image33_first_features);
    parseJson(image34_s ,&image34_C1 ,&image34_C2 , &image34_first_features);
    parseJson(image35_s ,&image35_C1 ,&image35_C2 , &image35_first_features);
    parseJson(image36_s ,&image36_C1 ,&image36_C2 , &image36_first_features);
    parseJson(image37_s ,&image37_C1 ,&image37_C2 , &image37_first_features);
    parseJson(image38_s ,&image38_C1 ,&image38_C2 , &image38_first_features);
    parseJson(image39_s ,&image39_C1 ,&image39_C2 , &image39_first_features);
    parseJson(image40_s ,&image40_C1 ,&image40_C2 , &image40_first_features);
    parseJson(image41_s ,&image41_C1 ,&image41_C2 , &image41_first_features);
    parseJson(image42_s ,&image42_C1 ,&image42_C2 , &image42_first_features);
    parseJson(image43_s ,&image43_C1 ,&image43_C2 , &image43_first_features);
    parseJson(image44_s ,&image44_C1 ,&image44_C2 , &image44_first_features);
    parseJson(image45_s ,&image45_C1 ,&image45_C2 , &image45_first_features);
    parseJson(image46_s ,&image46_C1 ,&image46_C2 , &image46_first_features);
    parseJson(image47_s ,&image47_C1 ,&image47_C2 , &image47_first_features);
    parseJson(image48_s ,&image48_C1 ,&image48_C2 , &image48_first_features);
    parseJson(image49_s ,&image49_C1 ,&image49_C2 , &image49_first_features);
    parseJson(image50_s ,&image50_C1 ,&image50_C2 , &image50_first_features);

    string image13_path = "images/100000797.jpg";
    images_path.push_back(image13_path);
    string image14_path = "images/100000827.jpg";
    images_path.push_back(image14_path);
    string image15_path = "images/100000857.jpg";
    images_path.push_back(image15_path);
    string image16_path = "images/100000887.jpg";
    images_path.push_back(image16_path);
    string image17_path = "images/100000917.jpg";
    images_path.push_back(image17_path);
    string image18_path = "images/100000947.jpg";
    images_path.push_back(image18_path);
    string image19_path = "images/100000977.jpg";
    images_path.push_back(image19_path);
    string image20_path = "images/100001007.jpg";
    images_path.push_back(image20_path);
    string image21_path = "images/100001037.jpg";
    images_path.push_back(image21_path);
    string image22_path = "images/100001067.jpg";
    images_path.push_back(image22_path);
    string image23_path = "images/100001097.jpg";
    images_path.push_back(image23_path);
    string image24_path = "images/100001127.jpg";
    images_path.push_back(image24_path);
    string image25_path = "images/100001157.jpg";
    images_path.push_back(image25_path);
    string image26_path="images/100001187.jpg";
    string image27_path="images/100001217.jpg";
    string image28_path="images/100001247.jpg";
    string image29_path="images/100001277.jpg";
    string image30_path="images/100001307.jpg";
    string image31_path="images/100001337.jpg";
    string image32_path="images/100001367.jpg";
    string image33_path="images/100001397.jpg";
    string image34_path="images/100001427.jpg";
    string image35_path="images/100001457.jpg";
    string image36_path="images/100001487.jpg";
    string image37_path="images/100001517.jpg";
    string image38_path="images/100001547.jpg";
    string image39_path="images/100001577.jpg";
    string image40_path="images/100001607.jpg";
    string image41_path="images/100001637.jpg";
    string image42_path="images/100001667.jpg";
    string image43_path="images/100001697.jpg";
    string image44_path="images/100001727.jpg";
    string image45_path="images/100001757.jpg";
    string image46_path="images/100001787.jpg";
    string image47_path="images/100001817.jpg";
    string image48_path="images/100001847.jpg";
    string image49_path="images/100001877.jpg";
    string image50_path="images/100001907.jpg";
    images_path.push_back(image26_path);
    images_path.push_back(image27_path);
    images_path.push_back(image28_path);
    images_path.push_back(image29_path);
    images_path.push_back(image30_path);
    images_path.push_back(image31_path);
    images_path.push_back(image32_path);
    images_path.push_back(image33_path);
    images_path.push_back(image34_path);
    images_path.push_back(image35_path);
    images_path.push_back(image36_path);
    images_path.push_back(image37_path);
    images_path.push_back(image38_path);
    images_path.push_back(image39_path);
    images_path.push_back(image40_path);
    images_path.push_back(image41_path);
    images_path.push_back(image42_path);
    images_path.push_back(image43_path);
    images_path.push_back(image44_path);
    images_path.push_back(image45_path);
    images_path.push_back(image46_path);
    images_path.push_back(image47_path);
    images_path.push_back(image48_path);
    images_path.push_back(image49_path);
    images_path.push_back(image50_path);



    Frame frame1(image1_C1,image1_C2);
    frames_vector.push_back(frame1);
    Frame frame2(image2_C1,image2_C2);
    frames_vector.push_back(frame2);

    Frame frame3(image3_C1,image3_C2);
    frames_vector.push_back(frame3);

    Frame frame4(image4_C1,image4_C2);
    frames_vector.push_back(frame4);

    Frame frame5(image5_C1,image5_C2);
    frames_vector.push_back(frame5);

    Frame frame6(image6_C1,image6_C2);
    frames_vector.push_back(frame6);

    Frame frame7(image7_C1,image7_C2);
    frames_vector.push_back(frame7);

    Frame frame8(image8_C1,image8_C2);
    frames_vector.push_back(frame8);

    Frame frame9(image9_C1,image9_C2);
    frames_vector.push_back(frame9);

    Frame frame10(image10_C1,image10_C2);
    frames_vector.push_back(frame10);

    Frame frame11(image11_C1,image11_C2);
    frames_vector.push_back(frame11);

    Frame frame12(image12_C1,image12_C2);
    frames_vector.push_back(frame12);

    Frame frame13(image13_C1,image13_C2);
    frames_vector.push_back(frame13);
    Frame frame14(image14_C1,image14_C2);
    frames_vector.push_back(frame14);
    Frame frame15(image15_C1,image15_C2);
    frames_vector.push_back(frame15);
    Frame frame16(image16_C1,image16_C2);
    frames_vector.push_back(frame16);
    Frame frame17(image17_C1,image17_C2);
    frames_vector.push_back(frame17);
    Frame frame18(image18_C1,image18_C2);
    frames_vector.push_back(frame18);
    Frame frame19(image19_C1,image19_C2);
    frames_vector.push_back(frame19);
    Frame frame20(image20_C1,image20_C2);
    frames_vector.push_back(frame20);
    Frame frame21(image21_C1,image21_C2);
    frames_vector.push_back(frame21);
    Frame frame22(image22_C1,image22_C2);
    frames_vector.push_back(frame22);
    Frame frame23(image23_C1,image23_C2);
    frames_vector.push_back(frame23);
    Frame frame24(image24_C1,image24_C2);
    frames_vector.push_back(frame24);
    Frame frame25(image25_C1,image25_C2);
    frames_vector.push_back(frame25);
    Frame frame26(image26_C1,image26_C2);
    Frame frame27(image27_C1,image27_C2);
    Frame frame28(image28_C1,image28_C2);
    Frame frame29(image29_C1,image29_C2);
    Frame frame30(image30_C1,image30_C2);
    Frame frame31(image31_C1,image31_C2);
    Frame frame32(image32_C1,image32_C2);
    Frame frame33(image33_C1,image33_C2);
    Frame frame34(image34_C1,image34_C2);
    Frame frame35(image35_C1,image35_C2);
    Frame frame36(image36_C1,image36_C2);
    Frame frame37(image37_C1,image37_C2);
    Frame frame38(image38_C1,image38_C2);
    Frame frame39(image39_C1,image39_C2);
    Frame frame40(image40_C1,image40_C2);
    Frame frame41(image41_C1,image41_C2);
    Frame frame42(image42_C1,image42_C2);
    Frame frame43(image43_C1,image43_C2);
    Frame frame44(image44_C1,image44_C2);
    Frame frame45(image45_C1,image45_C2);
    Frame frame46(image46_C1,image46_C2);
    Frame frame47(image47_C1,image47_C2);
    Frame frame48(image48_C1,image48_C2);
    Frame frame49(image49_C1,image49_C2);
    Frame frame50(image50_C1,image50_C2);
    frames_vector.push_back(frame26);
    frames_vector.push_back(frame27);
    frames_vector.push_back(frame28);
    frames_vector.push_back(frame29);
    frames_vector.push_back(frame30);
    frames_vector.push_back(frame31);
    frames_vector.push_back(frame32);
    frames_vector.push_back(frame33);
    frames_vector.push_back(frame34);
    frames_vector.push_back(frame35);
    frames_vector.push_back(frame36);
    frames_vector.push_back(frame37);
    frames_vector.push_back(frame38);
    frames_vector.push_back(frame39);
    frames_vector.push_back(frame40);
    frames_vector.push_back(frame41);
    frames_vector.push_back(frame42);
    frames_vector.push_back(frame43);
    frames_vector.push_back(frame44);
    frames_vector.push_back(frame45);
    frames_vector.push_back(frame46);
    frames_vector.push_back(frame47);
    frames_vector.push_back(frame48);
    frames_vector.push_back(frame49);
    frames_vector.push_back(frame50);



    /// pushing all features to the features vector
    features_vector.push_back(image1_first_features);
    features_vector.push_back(image2_first_features);
    features_vector.push_back(image3_first_features);
    features_vector.push_back(image4_first_features);
    features_vector.push_back(image5_first_features);
    features_vector.push_back(image6_first_features);
    features_vector.push_back(image7_first_features);
    features_vector.push_back(image8_first_features);
    features_vector.push_back(image9_first_features);
    features_vector.push_back(image10_first_features);
    features_vector.push_back(image11_first_features);
    features_vector.push_back(image12_first_features);
    features_vector.push_back(image13_first_features);
    features_vector.push_back(image14_first_features);
    features_vector.push_back(image15_first_features);
    features_vector.push_back(image16_first_features);
    features_vector.push_back(image17_first_features);
    features_vector.push_back(image18_first_features);
    features_vector.push_back(image19_first_features);
    features_vector.push_back(image20_first_features);
    features_vector.push_back(image21_first_features);
    features_vector.push_back(image22_first_features);
    features_vector.push_back(image23_first_features);
    features_vector.push_back(image24_first_features);
    features_vector.push_back(image25_first_features);
    features_vector.push_back(image26_first_features);
    features_vector.push_back(image27_first_features);
    features_vector.push_back(image28_first_features);
    features_vector.push_back(image29_first_features);
    features_vector.push_back(image30_first_features);
    features_vector.push_back(image31_first_features);
    features_vector.push_back(image32_first_features);
    features_vector.push_back(image33_first_features);
    features_vector.push_back(image34_first_features);
    features_vector.push_back(image35_first_features);
    features_vector.push_back(image36_first_features);
    features_vector.push_back(image37_first_features);
    features_vector.push_back(image38_first_features);
    features_vector.push_back(image39_first_features);
    features_vector.push_back(image40_first_features);
    features_vector.push_back(image41_first_features);
    features_vector.push_back(image42_first_features);
    features_vector.push_back(image43_first_features);
    features_vector.push_back(image44_first_features);
    features_vector.push_back(image45_first_features);
    features_vector.push_back(image46_first_features);
    features_vector.push_back(image47_first_features);
    features_vector.push_back(image48_first_features);
    features_vector.push_back(image49_first_features);
    features_vector.push_back(image50_first_features);
}

/*!
 * function for creating a grid from a given frame, calculating its center from the average of all the features of the
 * frame, and get his cam_ori, R & T.
 * @param frame_features - the features of the given frame (we calculate the grid center with it)
 * @param given_frame - the given frame we're using to create the grid
 * @param step_size - the step size of the grid.
 * @return - the created grid.
 */
grid3D createGridFromGivenFrame(vector<worldPoint>& frame_features, Frame& given_frame, float step_size){
    Matx31f point_grid_center = calc_avg_grid_center(frame_features);
    grid3D grid(step_size, given_frame.getCamori(), point_grid_center, given_frame.getR(), given_frame.getT());
    return grid;
}

/*!
 * function for creating a vector that contains all the grid's edges
 * @param vec - the vector contatining all the grid edges
 */
void createGridEdges(vector<Point3d>& vec){
    Point3d p1(-EDGE_COORDINATE,-EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p3(-EDGE_COORDINATE,EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p2(EDGE_COORDINATE,-EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p4(EDGE_COORDINATE,EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p5(-EDGE_COORDINATE,-EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p6(EDGE_COORDINATE,-EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p7(-EDGE_COORDINATE,EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p8(EDGE_COORDINATE,EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p9(0,0,0);
    vec.push_back(p1);
    vec.push_back(p2);
    vec.push_back(p3);
    vec.push_back(p4);
    vec.push_back(p5);
    vec.push_back(p6);
    vec.push_back(p7);
    vec.push_back(p8);
    vec.push_back(p9);

}
/*!
 * function for projecting the grid voxels into all the frames (the voxels of the grid are only the one that have been blacked by a ray)
 * @param grid - the grid we project
 * @param grid_edges - a vector contating all the coordinates of the grid's edges
 * @param frames - the frames we projct the grid to.
 * @param images_path - paths to all the images in the local machine.
 */
void projectGridToFrames(grid3D& grid, vector<Point3d>& grid_edges, vector<Frame>& frames, vector<string>& images_path){
    int cntr=0;
    grid.projectpointsFromGridToImage(images_path[0], grid_edges, grid.getGrid(), &frames[0], "Frame 1 - made by", &cntr);
    grid.projectpointsFromGridToImage(images_path[1], grid_edges, grid.getGrid(), &frames[1], "Frame 2", &cntr);
    grid.projectpointsFromGridToImage(images_path[2], grid_edges, grid.getGrid(), &frames[2], "Frame 3", &cntr);
    grid.projectpointsFromGridToImage(images_path[3], grid_edges, grid.getGrid(), &frames[3], "Frame 4", &cntr);
    grid.projectpointsFromGridToImage(images_path[4], grid_edges, grid.getGrid(), &frames[4], "Frame 5", &cntr);
    grid.projectpointsFromGridToImage(images_path[5], grid_edges, grid.getGrid(), &frames[5], "Frame 6", &cntr);
    grid.projectpointsFromGridToImage(images_path[6], grid_edges, grid.getGrid(), &frames[6], "Frame 7", &cntr);
    grid.projectpointsFromGridToImage(images_path[7], grid_edges, grid.getGrid(), &frames[7], "Frame 8", &cntr);
    grid.projectpointsFromGridToImage(images_path[8], grid_edges, grid.getGrid(), &frames[8], "Frame 9", &cntr);
    grid.projectpointsFromGridToImage(images_path[9], grid_edges, grid.getGrid(), &frames[9], "Frame 10", &cntr);
    grid.projectpointsFromGridToImage(images_path[10], grid_edges, grid.getGrid(), &frames[10], "Frame 11", &cntr);
    grid.projectpointsFromGridToImage(images_path[11], grid_edges, grid.getGrid(), &frames[11], "Frame 12", &cntr);
    grid.projectpointsFromGridToImage(images_path[12], grid_edges, grid.getGrid(), &frames[12], "Frame 13", &cntr);
    grid.projectpointsFromGridToImage(images_path[13], grid_edges, grid.getGrid(), &frames[13], "Frame 14", &cntr);
    grid.projectpointsFromGridToImage(images_path[14], grid_edges, grid.getGrid(), &frames[14], "Frame 15", &cntr);
    grid.projectpointsFromGridToImage(images_path[15], grid_edges, grid.getGrid(), &frames[15], "Frame 16", &cntr);
    grid.projectpointsFromGridToImage(images_path[16], grid_edges, grid.getGrid(), &frames[16], "Frame 17", &cntr);
    grid.projectpointsFromGridToImage(images_path[17], grid_edges, grid.getGrid(), &frames[17], "Frame 18", &cntr);
    grid.projectpointsFromGridToImage(images_path[18], grid_edges, grid.getGrid(), &frames[18], "Frame 19", &cntr);
    grid.projectpointsFromGridToImage(images_path[19], grid_edges, grid.getGrid(), &frames[19], "Frame 20", &cntr);
    grid.projectpointsFromGridToImage(images_path[20], grid_edges, grid.getGrid(), &frames[20], "Frame 21", &cntr);
    grid.projectpointsFromGridToImage(images_path[21], grid_edges, grid.getGrid(), &frames[21], "Frame 22", &cntr);
    grid.projectpointsFromGridToImage(images_path[22], grid_edges, grid.getGrid(), &frames[22], "Frame 23", &cntr);
    grid.projectpointsFromGridToImage(images_path[23], grid_edges, grid.getGrid(), &frames[23], "Frame 24", &cntr);
    grid.projectpointsFromGridToImage(images_path[24], grid_edges, grid.getGrid(), &frames[24], "Frame 25", &cntr);
    grid.projectpointsFromGridToImage(images_path[25], grid_edges, grid.getGrid(), &frames[25], "Frame 26", &cntr);
    grid.projectpointsFromGridToImage(images_path[26], grid_edges, grid.getGrid(), &frames[26], "Frame 27", &cntr);
    grid.projectpointsFromGridToImage(images_path[27], grid_edges, grid.getGrid(), &frames[27], "Frame 28", &cntr);
    grid.projectpointsFromGridToImage(images_path[28], grid_edges, grid.getGrid(), &frames[28], "Frame 29", &cntr);
    grid.projectpointsFromGridToImage(images_path[29], grid_edges, grid.getGrid(), &frames[29], "Frame 30", &cntr);
    grid.projectpointsFromGridToImage(images_path[30], grid_edges, grid.getGrid(), &frames[30], "Frame 31", &cntr);
    grid.projectpointsFromGridToImage(images_path[31], grid_edges, grid.getGrid(), &frames[31], "Frame 32", &cntr);
    grid.projectpointsFromGridToImage(images_path[32], grid_edges, grid.getGrid(), &frames[32], "Frame 33", &cntr);
    grid.projectpointsFromGridToImage(images_path[33], grid_edges, grid.getGrid(), &frames[33], "Frame 34", &cntr);
    grid.projectpointsFromGridToImage(images_path[34], grid_edges, grid.getGrid(), &frames[34], "Frame 35", &cntr);
    grid.projectpointsFromGridToImage(images_path[35], grid_edges, grid.getGrid(), &frames[35], "Frame 36", &cntr);
    grid.projectpointsFromGridToImage(images_path[36], grid_edges, grid.getGrid(), &frames[36], "Frame 37", &cntr);
    grid.projectpointsFromGridToImage(images_path[37], grid_edges, grid.getGrid(), &frames[37], "Frame 38", &cntr);
    grid.projectpointsFromGridToImage(images_path[38], grid_edges, grid.getGrid(), &frames[38], "Frame 39", &cntr);
    grid.projectpointsFromGridToImage(images_path[39], grid_edges, grid.getGrid(), &frames[39], "Frame 40", &cntr);
    grid.projectpointsFromGridToImage(images_path[40], grid_edges, grid.getGrid(), &frames[40], "Frame 41", &cntr);
    grid.projectpointsFromGridToImage(images_path[41], grid_edges, grid.getGrid(), &frames[41], "Frame 42", &cntr);
    grid.projectpointsFromGridToImage(images_path[42], grid_edges, grid.getGrid(), &frames[42], "Frame 43", &cntr);
    grid.projectpointsFromGridToImage(images_path[43], grid_edges, grid.getGrid(), &frames[43], "Frame 44", &cntr);
    grid.projectpointsFromGridToImage(images_path[44], grid_edges, grid.getGrid(), &frames[44], "Frame 45", &cntr);
    grid.projectpointsFromGridToImage(images_path[45], grid_edges, grid.getGrid(), &frames[45], "Frame 46", &cntr);
    grid.projectpointsFromGridToImage(images_path[46], grid_edges, grid.getGrid(), &frames[46], "Frame 47", &cntr);
    grid.projectpointsFromGridToImage(images_path[47], grid_edges, grid.getGrid(), &frames[47], "Frame 48", &cntr);
    grid.projectpointsFromGridToImage(images_path[48], grid_edges, grid.getGrid(), &frames[48], "Frame 49", &cntr);
    grid.projectpointsFromGridToImage(images_path[49], grid_edges, grid.getGrid(), &frames[49], "Frame 50", &cntr);
}

/*!
 * function for creating a visualiztion of the confidence of all the voxels, the bigger the confidence of the voxel,
 * it will be presented by a bigger circle in the image, we do it for 50 diffrent frames.
 * @param grid - the grid we project
 * @param grid_edges - a vector contating all the coordinates of the grid's edges
 * @param frames - the frames we projct the grid to.
 * @param images_path - paths to all the images in the local machine.
 */
void createConfidenceHeatMap(grid3D& grid, vector<Point3d>& grid_edges, vector<Frame>& frames, vector<string>& images_path){
    int cntr=0;
    grid.createConfidenceHeatMap(images_path[0], grid_edges, grid.getGrid(), &frames[0], "Frame 1 - made by", &cntr);
    grid.createConfidenceHeatMap(images_path[1], grid_edges, grid.getGrid(), &frames[1], "Frame 2", &cntr);
    grid.createConfidenceHeatMap(images_path[2], grid_edges, grid.getGrid(), &frames[2], "Frame 3", &cntr);
    grid.createConfidenceHeatMap(images_path[3], grid_edges, grid.getGrid(), &frames[3], "Frame 4", &cntr);
    grid.createConfidenceHeatMap(images_path[4], grid_edges, grid.getGrid(), &frames[4], "Frame 5", &cntr);
    grid.createConfidenceHeatMap(images_path[5], grid_edges, grid.getGrid(), &frames[5], "Frame 6", &cntr);
    grid.createConfidenceHeatMap(images_path[6], grid_edges, grid.getGrid(), &frames[6], "Frame 7", &cntr);
    grid.createConfidenceHeatMap(images_path[7], grid_edges, grid.getGrid(), &frames[7], "Frame 8", &cntr);
    grid.createConfidenceHeatMap(images_path[8], grid_edges, grid.getGrid(), &frames[8], "Frame 9", &cntr);
    grid.createConfidenceHeatMap(images_path[9], grid_edges, grid.getGrid(), &frames[9], "Frame 10", &cntr);
    grid.createConfidenceHeatMap(images_path[10], grid_edges, grid.getGrid(), &frames[10], "Frame 11", &cntr);
    grid.createConfidenceHeatMap(images_path[11], grid_edges, grid.getGrid(), &frames[11], "Frame 12", &cntr);
    grid.createConfidenceHeatMap(images_path[12], grid_edges, grid.getGrid(), &frames[12], "Frame 13", &cntr);
    grid.createConfidenceHeatMap(images_path[13], grid_edges, grid.getGrid(), &frames[13], "Frame 14", &cntr);
    grid.createConfidenceHeatMap(images_path[14], grid_edges, grid.getGrid(), &frames[14], "Frame 15", &cntr);
    grid.createConfidenceHeatMap(images_path[15], grid_edges, grid.getGrid(), &frames[15], "Frame 16", &cntr);
    grid.createConfidenceHeatMap(images_path[16], grid_edges, grid.getGrid(), &frames[16], "Frame 17", &cntr);
    grid.createConfidenceHeatMap(images_path[17], grid_edges, grid.getGrid(), &frames[17], "Frame 18", &cntr);
    grid.createConfidenceHeatMap(images_path[18], grid_edges, grid.getGrid(), &frames[18], "Frame 19", &cntr);
    grid.createConfidenceHeatMap(images_path[19], grid_edges, grid.getGrid(), &frames[19], "Frame 20", &cntr);
    grid.createConfidenceHeatMap(images_path[20], grid_edges, grid.getGrid(), &frames[20], "Frame 21", &cntr);
    grid.createConfidenceHeatMap(images_path[21], grid_edges, grid.getGrid(), &frames[21], "Frame 22", &cntr);
    grid.createConfidenceHeatMap(images_path[22], grid_edges, grid.getGrid(), &frames[22], "Frame 23", &cntr);
    grid.createConfidenceHeatMap(images_path[23], grid_edges, grid.getGrid(), &frames[23], "Frame 24", &cntr);
    grid.createConfidenceHeatMap(images_path[24], grid_edges, grid.getGrid(), &frames[24], "Frame 25", &cntr);
    grid.createConfidenceHeatMap(images_path[25], grid_edges, grid.getGrid(), &frames[25], "Frame 26", &cntr);
    grid.createConfidenceHeatMap(images_path[26], grid_edges, grid.getGrid(), &frames[26], "Frame 27", &cntr);
    grid.createConfidenceHeatMap(images_path[27], grid_edges, grid.getGrid(), &frames[27], "Frame 28", &cntr);
    grid.createConfidenceHeatMap(images_path[28], grid_edges, grid.getGrid(), &frames[28], "Frame 29", &cntr);
    grid.createConfidenceHeatMap(images_path[29], grid_edges, grid.getGrid(), &frames[29], "Frame 30", &cntr);
    grid.createConfidenceHeatMap(images_path[30], grid_edges, grid.getGrid(), &frames[30], "Frame 31", &cntr);
    grid.createConfidenceHeatMap(images_path[31], grid_edges, grid.getGrid(), &frames[31], "Frame 32", &cntr);
    grid.createConfidenceHeatMap(images_path[32], grid_edges, grid.getGrid(), &frames[32], "Frame 33", &cntr);
    grid.createConfidenceHeatMap(images_path[33], grid_edges, grid.getGrid(), &frames[33], "Frame 34", &cntr);
    grid.createConfidenceHeatMap(images_path[34], grid_edges, grid.getGrid(), &frames[34], "Frame 35", &cntr);
    grid.createConfidenceHeatMap(images_path[35], grid_edges, grid.getGrid(), &frames[35], "Frame 36", &cntr);
    grid.createConfidenceHeatMap(images_path[36], grid_edges, grid.getGrid(), &frames[36], "Frame 37", &cntr);
    grid.createConfidenceHeatMap(images_path[37], grid_edges, grid.getGrid(), &frames[37], "Frame 38", &cntr);
    grid.createConfidenceHeatMap(images_path[38], grid_edges, grid.getGrid(), &frames[38], "Frame 39", &cntr);
    grid.createConfidenceHeatMap(images_path[39], grid_edges, grid.getGrid(), &frames[39], "Frame 40", &cntr);
    grid.createConfidenceHeatMap(images_path[40], grid_edges, grid.getGrid(), &frames[40], "Frame 41", &cntr);
    grid.createConfidenceHeatMap(images_path[41], grid_edges, grid.getGrid(), &frames[41], "Frame 42", &cntr);
    grid.createConfidenceHeatMap(images_path[42], grid_edges, grid.getGrid(), &frames[42], "Frame 43", &cntr);
    grid.createConfidenceHeatMap(images_path[43], grid_edges, grid.getGrid(), &frames[43], "Frame 44", &cntr);
    grid.createConfidenceHeatMap(images_path[44], grid_edges, grid.getGrid(), &frames[44], "Frame 45", &cntr);
    grid.createConfidenceHeatMap(images_path[45], grid_edges, grid.getGrid(), &frames[45], "Frame 46", &cntr);
    grid.createConfidenceHeatMap(images_path[46], grid_edges, grid.getGrid(), &frames[46], "Frame 47", &cntr);
    grid.createConfidenceHeatMap(images_path[47], grid_edges, grid.getGrid(), &frames[47], "Frame 48", &cntr);
    grid.createConfidenceHeatMap(images_path[48], grid_edges, grid.getGrid(), &frames[48], "Frame 49", &cntr);
    grid.createConfidenceHeatMap(images_path[49], grid_edges, grid.getGrid(), &frames[49], "Frame 50", &cntr);
}


void createColorMap(string path_to_img){
    Mat im1 = imread(path_to_img);

}


/*!
 * function to create the video from all the images
 */
void video ()
{
    VideoWriter out_capture("/Users/elongrubman/Desktop/video.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 3,
                            Size(1920, 1440));
    for (int i = 0; i < 50; i++) {

        VideoCapture in_capture("/Users/elongrubman/Desktop/outputs/"+to_string(i)+".jpg");
        Mat img;

        while (true) {
            in_capture >> img;
            if (img.empty())
                break;

            out_capture.write(img);
        }
    }
}
/*!
 * function that get voxel's confidance and return his RGB value. the RGB value is more red if the confidence bigger
 * @param confidence
 * @param RGBvalues
 */
vector<int> getRGBbyConfidance(float confidence){
    vector<int> RGBvalues;
    RGBvalues.push_back(255 - (255 * confidence));
    RGBvalues.push_back(255 - (2 * 255 * abs(confidence - 0.5)));
    RGBvalues.push_back(255 * confidence);
    return RGBvalues;
}

/*!
 * function that get voxels indexs, convert them to coordinates in the word and put them in vector who represent voxel details
 * in grid.txt file
 * @param indexs - voxel indexs in gris axis
 * @param line - vector who represent detaile on some voxel
 * @param grid - the data structure when the black voxels exist
 */
vector<float> convertVoxelIndexToCoordinateInTheWorld(tuple<int,int,int> indexs, grid3D& grid){
    vector<float> XYZpoints;
    Matx31f grid_coordinates_in_world = grid.mapFromGridToWorld(get<0>(indexs), get<1>(indexs), get<2>(indexs));
    XYZpoints.push_back(grid_coordinates_in_world(0,0));
    XYZpoints.push_back(grid_coordinates_in_world(1,0));
    XYZpoints.push_back(grid_coordinates_in_world(2,0));
    return XYZpoints;
}

/*!
 * function that generate grid.ply with all the details of black voxels
 * @param grid - the data structure when the black voxels exist
 */
void exportGridToFile(grid3D& grid){

    std::ofstream outFile("grid_ply/grid.ply");
    outFile << "ply" << endl;
    outFile << "format ascii 1.0" << endl;
    outFile << "element vertex " + to_string(grid.getGrid().size()) << endl;
    outFile << "property float x" << endl;
    outFile << "property float y" << endl;
    outFile << "property float z" << endl;
    outFile << "property float nx" << endl;
    outFile << "property float ny" << endl;
    outFile << "property float nz" << endl;
    outFile << "property uchar red" << endl;
    outFile << "property uchar green" << endl;
    outFile << "property uchar blue" << endl;
    outFile << "end_header" << endl;

    vector<float > XYZpoints;
    vector<int> RGBvalues;
    for (auto it = grid.getGrid().begin(); it != grid.getGrid().end() ; it++) {
        if(it->second.getConfidence() >= THRESHOLD_CONFIDENCE) {
            XYZpoints = convertVoxelIndexToCoordinateInTheWorld(it->first, grid);
            RGBvalues = getRGBbyConfidance(it->second.getConfidence());
            outFile << to_string(XYZpoints[0]) << " " << to_string(XYZpoints[1]) << " " << to_string(XYZpoints[2])
                    << " 0 0 0 "
                    << to_string(RGBvalues[0]) << " " << to_string(RGBvalues[1]) << " " << to_string(RGBvalues[2])
                    << endl;
            XYZpoints.clear();
            RGBvalues.clear();
        }

    }
    outFile.close();
}

/*!
 * function that find maximum confidence value between the black voxels and normalize the voxels's confidende value by
 * division by the maximum value. (turn all confidence values to [0,1] scale)
 * @param grid - the data structure when the black voxels exist
 */
void normalizeVoxelsConfidence(grid3D& grid){
    int counter = 0;
    float max_confidence=-9999;
    for (auto it = grid.getGrid().begin(); it != grid.getGrid().end(); ++it) {
        if(it->second.getConfidence() > max_confidence) {
            max_confidence = it->second.getConfidence();
        }
    }
    for (auto it = grid.getGrid().begin(); it != grid.getGrid().end(); ++it) {
        float normalize_confidence = it->second.getConfidence() / max_confidence;
        it->second.setConfidence(normalize_confidence);
    }

    cout << "finished_normalizing_confidence" << endl;

}

    void createConfigurationFile(string path, grid3D& grid, int frame_num, int number_of_features_from_frame, bool all,
            int number_of_meshes_created){
        std::ofstream myfile;
        myfile.open(path);
        myfile << "****** Configuration File ******" << '\n';
        myfile << "grid size: " << GRID_SIZE << '\n';
        myfile << "number of voxels: " << grid.getGrid().size() << '\n';
        myfile << "densified grid size:  " << grid.getDensifiedGrid().size() << '\n';
        myfile << "grid step size:  " << grid.getStepSize() << '\n';
        myfile << "confidence threshold:  " << THRESHOLD_CONFIDENCE << '\n';
        myfile << "sigma:  " << SIGMA << '\n';
        myfile << "alpha:  " << ALPHA << '\n';
        myfile << "number of frames presented:  " << frame_num << '\n';
        if(all){
            myfile << "number of features from each frame:  " << "all" << '\n';
        }
        else{
            myfile << "number of features from each frame:  " << number_of_features_from_frame << '\n';
        }
        myfile << "number of meshes created:  " << number_of_meshes_created << '\n';
        myfile.close();



    }


int main(int argc, char **argv){
//    testy();
    /// test of creating a ball
//    grid3D grid_for_ball;
//    createBall(grid_for_ball);



    vector<string> images_paths;
    vector<vector<worldPoint>> frames_features;
    vector<Frame> frames_vector;
    /// parse jsons
    ParseAllJsons(images_paths, frames_vector, frames_features);
    /// create grid
    grid3D grid = createGridFromGivenFrame(frames_features[0], frames_vector[0], 0.015);
    vector<Point3d> grid_edges;
    /// create grid edges
    createGridEdges(grid_edges);

    /// creating cam poses for all the frames in the grid axis
    vector<Matx31f> cam_poses;
    createCamPoseVectorForFrames(cam_poses,frames_vector, grid);

//   testForChackTheDistance(grid);



    /// presenting twenty five rays from all frames
//    testPresentTwentyFiveRaysWithSameFeature(cam_poses, frames_features, grid);
    int number_of_frames_to_present = 50;
    CreateRaysAndVoxels(cam_poses, frames_features, grid, number_of_frames_to_present);

    /// convert confidence values to [0,1] scale by normalize them
    normalizeVoxelsConfidence(grid);


    cout << "start update grid by confidence" << endl;
    grid.updateGridByConfidenceThreshold();
    cout << "end update grid by confidence" << endl;
    cout << "number of voxels is " << grid.getGrid().size() << endl;
  //  grid.printGridByConfidence();
 //   createConfidenceHeatMap(grid, grid_edges, frames_vector, images_paths);

  //  grid.printGridByConfidence();
    /// create grid.ply file for Mesh Lab
    exportGridToFile(grid);

    /// export grid to a text file
    vector<vector<float>> lines;
    grid.createLines(lines);
    grid.createTextFileFromGrid("grid_text/grid_as_text.txt", lines);
    cout << " start densify" << endl;
    /// densify the grid's data
    grid.addVoxelsToDensifiedGrid();
    cout << " finished densify" << endl;
    /// create grid cell map
    grid.createGridCellMap();
    cout << " start create grid cell map" << endl;
    cout << " finished create grid cell map" << endl;
    /// create mesh from grid
    vector<Mesh> grid_mesh;
    cout << " start create mesh from grid" << endl;
    CreateMeshFromGrid(grid_mesh, grid);
    cout << " finished create mesh from grid" << endl;
    /// create full mesh
    Mesh full_mesh;
//    cout << " start create full mesh" << endl;
    createFullMesh(grid_mesh, full_mesh);
    cout << " finished create full mesh" << endl;
    /// export mesh to file
    string path_to_file("grid_as_mesh/grid_as_mesh.off");
    cout << " start exporting mesh to file " << endl;
    exportMeshToFile(full_mesh, path_to_file);
    cout << " finished exporting mesh to file " << endl;
    cout << "create configuration file" << endl;
    string path_to_configuration_file("configuration_file/configuration_file.txt");
    createConfigurationFile(path_to_configuration_file, grid, number_of_frames_to_present, FEATURES_NUM_TO_PRESENT,
            true, grid_mesh.size());





//    video();

    return 0;
}


