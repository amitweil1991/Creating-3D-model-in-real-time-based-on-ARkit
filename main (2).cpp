#include <nlohmann/json.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Frame.h"
#include "grid3D.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


//using namespace cv;
//using namespace std;
using json = nlohmann::json;

void video ();
void testProjection(vector<worldPoint> image1_first_features, vector<string> imagesPath,vector<Frame> frames, int* cntr);
Matx31f calc_avg_grid_center(vector<worldPoint> features_point);


int main(int argc, char **argv){

    int cntr=0;

    vector<string> images_path;
    /// Json file that the Grid build from
    vector<float> image1_C1;
    vector<float> image1_C2;
    vector<worldPoint> image1_first_features;
    string image1_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000437.json";
    parseJson(image1_s ,&image1_C1 ,&image1_C2 , &image1_first_features);
    string image1_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000437.jpg";
    images_path.push_back(image1_path);

    vector<float> image2_C1;
    vector<float> image2_C2;
    vector<worldPoint> image2_first_features;
    string image2_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000467.json";
    parseJson(image2_s ,&image2_C1 ,&image2_C2 , &image2_first_features);
    string image2_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000467.jpg";
    images_path.push_back(image2_path);


    vector<float> image3_C1;
    vector<float> image3_C2;
    vector<worldPoint> image3_first_features;
    string image3_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000497.json";
    parseJson(image3_s ,&image3_C1 ,&image3_C2 , &image3_first_features);
    string image3_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000497.jpg";
    images_path.push_back(image3_path);

    vector<float> image4_C1;
    vector<float> image4_C2;
    vector<worldPoint> image4_first_features;
    string image4_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000527.json";
    parseJson(image4_s ,&image4_C1 ,&image4_C2 , &image4_first_features);
    string image4_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000527.jpg";
    images_path.push_back(image4_path);

    vector<float> image5_C1;
    vector<float> image5_C2;
    vector<worldPoint> image5_first_features;
    string image5_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000557.json";
    parseJson(image5_s ,&image5_C1 ,&image5_C2 , &image5_first_features);
    string image5_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000557.jpg";
    images_path.push_back(image5_path);

    vector<float> image6_C1;
    vector<float> image6_C2;
    vector<worldPoint> image6_first_features;
    string image6_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000587.json";
    parseJson(image6_s ,&image6_C1 ,&image6_C2 , &image6_first_features);
    string image6_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000587.jpg";
    images_path.push_back(image6_path);

    vector<float> image7_C1;
    vector<float> image7_C2;
    vector<worldPoint> image7_first_features;
    string image7_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000617.json";
    parseJson(image7_s ,&image7_C1 ,&image7_C2 , &image7_first_features);
    string image7_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000617.jpg";
    images_path.push_back(image7_path);

    vector<float> image8_C1;
    vector<float> image8_C2;
    vector<worldPoint> image8_first_features;
    string image8_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000647.json";
    parseJson(image8_s ,&image8_C1 ,&image8_C2 , &image8_first_features);
    string image8_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000647.jpg";
    images_path.push_back(image8_path);

    vector<float> image9_C1;
    vector<float> image9_C2;
    vector<worldPoint> image9_first_features;
    string image9_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000677.json";
    parseJson(image9_s ,&image9_C1 ,&image9_C2 , &image9_first_features);
    string image9_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000677.jpg";
    images_path.push_back(image9_path);

    vector<float> image10_C1;
    vector<float> image10_C2;
    vector<worldPoint> image10_first_features;
    string image10_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000707.json";
    parseJson(image10_s ,&image10_C1 ,&image10_C2 , &image10_first_features);
    string image10_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000707.jpg";
    images_path.push_back(image10_path);

    vector<float> image11_C1;
    vector<float> image11_C2;
    vector<worldPoint> image11_first_features;
    string image11_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000737.json";
    parseJson(image11_s ,&image11_C1 ,&image11_C2 , &image11_first_features);
    string image11_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000737.jpg";
    images_path.push_back(image11_path);

    vector<float> image12_C1;
    vector<float> image12_C2;
    vector<worldPoint> image12_first_features;
    string image12_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000767.json";
    parseJson(image12_s ,&image12_C1 ,&image12_C2 , &image12_first_features);
    string image12_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000767.jpg";
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

    string image13_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000797.json";
    string image14_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000827.json";
    string image15_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000857.json";
    string image16_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000887.json";
    string image17_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000917.json";
    string image18_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000947.json";
    string image19_s="/Users/elongrubman/Desktop/range_rover_scan_1/100000977.json";
    string image20_s="/Users/elongrubman/Desktop/range_rover_scan_1/100001007.json";
    string image21_s="/Users/elongrubman/Desktop/range_rover_scan_1/100001037.json";
    string image22_s="/Users/elongrubman/Desktop/range_rover_scan_1/100001067.json";
    string image23_s="/Users/elongrubman/Desktop/range_rover_scan_1/100001097.json";
    string image24_s="/Users/elongrubman/Desktop/range_rover_scan_1/100001127.json";
    string image25_s="/Users/elongrubman/Desktop/range_rover_scan_1/100001157.json";

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

    string image13_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000797.jpg";
    images_path.push_back(image13_path);
    string image14_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000827.jpg";
    images_path.push_back(image14_path);
    string image15_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000857.jpg";
    images_path.push_back(image15_path);
    string image16_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000887.jpg";
    images_path.push_back(image16_path);
    string image17_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000917.jpg";
    images_path.push_back(image17_path);
    string image18_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000947.jpg";
    images_path.push_back(image18_path);
    string image19_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100000977.jpg";
    images_path.push_back(image19_path);
    string image20_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100001007.jpg";
    images_path.push_back(image20_path);
    string image21_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100001037.jpg";
    images_path.push_back(image21_path);
    string image22_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100001067.jpg";
    images_path.push_back(image22_path);
    string image23_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100001097.jpg";
    images_path.push_back(image23_path);
    string image24_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100001127.jpg";
    images_path.push_back(image24_path);
    string image25_path = "/Users/elongrubman/Desktop/range_rover_scan_1/100001157.jpg";
    images_path.push_back(image25_path);



    vector<Frame> frames_vector;
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



    Matx31f point_grid_center = calc_avg_grid_center(image1_first_features);
    grid3D grid(0.015, frame1.getCamori(), point_grid_center);

    Point3d p1(-EDGE_COORDINATE,-EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p3(-EDGE_COORDINATE,EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p2(EDGE_COORDINATE,-EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p4(EDGE_COORDINATE,EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p5(-EDGE_COORDINATE,-EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p6(EDGE_COORDINATE,-EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p7(-EDGE_COORDINATE,EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p8(EDGE_COORDINATE,EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p9(0,0,0);

    vector<Point3d> vec;
    vec.push_back(p1);
    vec.push_back(p2);
    vec.push_back(p3);
    vec.push_back(p4);
    vec.push_back(p5);
    vec.push_back(p6);
    vec.push_back(p7);
    vec.push_back(p8);
    vec.push_back(p9);


//    testProjection(image1_first_features, images_path, frames_vector, &cntr2);

    vector<voxel> v;
    Matx31f pw = grid.getPw();
    Matx31f camPose(0,0, 5);
    Matx31f point(0,0, 4);
    straight_line_equation l(camPose, point);
    Point2d slope = grid.getSlopeRange(l);

    cout << "Range of S: " << "(" << slope.x << " , " << slope.y << ")" << endl;

    tuple<Matx31f, Matx31f> intersection_points = grid.findIntersectionPoint(l,slope);
    Matx31f entrance_point = get<0>(intersection_points);
    vector<tuple<int, int, int>> keys = grid.getVoxelFromCoordinatesOrPush(entrance_point(0,0), entrance_point(1,0), entrance_point(2,0), l);
    grid.bresenhamAlgorithim(l, keys[0]);

    keys.clear();
    for (auto it = grid.getGrid().begin(); it != grid.getGrid().end(); it++) {
        tuple<int,int,int> key = it->first;
        keys.push_back(key);
        cout << get<0>(key) << ", " << get<1>(key) << ", " << get<2>(key) << endl;

    }

    grid.projectpointsFromGridToImage(image1_path,vec,keys,&frame1, "Frame 1 - made by",&cntr);
    grid.projectpointsFromGridToImage(image2_path,vec,keys,&frame2, "Frame 2",&cntr);
    grid.projectpointsFromGridToImage(image3_path,vec,keys,&frame3, "Frame 3", &cntr);
    grid.projectpointsFromGridToImage(image4_path,vec,keys,&frame4, "Frame 4",&cntr);
    grid.projectpointsFromGridToImage(image5_path,vec,keys,&frame5, "Frame 5", &cntr);
    grid.projectpointsFromGridToImage(image6_path,vec,keys,&frame6, "Frame 6", &cntr);
    grid.projectpointsFromGridToImage(image7_path,vec,keys,&frame7, "Frame 7", &cntr);
    grid.projectpointsFromGridToImage(image8_path,vec,keys,&frame8, "Frame 8", &cntr);
    grid.projectpointsFromGridToImage(image9_path,vec,keys,&frame9, "Frame 9", &cntr);
    grid.projectpointsFromGridToImage(image10_path,vec,keys,&frame10, "Frame 10", &cntr);
    grid.projectpointsFromGridToImage(image11_path,vec,keys,&frame11, "Frame 11", &cntr);
    grid.projectpointsFromGridToImage(image12_path,vec,keys,&frame12, "Frame 12", &cntr);
    grid.projectpointsFromGridToImage(image13_path,vec,keys,&frame13, "Frame 13", &cntr);
    grid.projectpointsFromGridToImage(image14_path,vec,keys,&frame14, "Frame 14", &cntr);
    grid.projectpointsFromGridToImage(image15_path,vec,keys,&frame15, "Frame 15", &cntr);
    grid.projectpointsFromGridToImage(image16_path,vec,keys,&frame16, "Frame 16", &cntr);
    grid.projectpointsFromGridToImage(image17_path,vec,keys,&frame17, "Frame 17", &cntr);
    grid.projectpointsFromGridToImage(image18_path,vec,keys,&frame18, "Frame 18", &cntr);
    grid.projectpointsFromGridToImage(image19_path,vec,keys,&frame19, "Frame 19", &cntr);
    grid.projectpointsFromGridToImage(image20_path,vec,keys,&frame20, "Frame 20", &cntr);
    grid.projectpointsFromGridToImage(image21_path,vec,keys,&frame21, "Frame 21", &cntr);
    grid.projectpointsFromGridToImage(image22_path,vec,keys,&frame22, "Frame 22", &cntr);
    grid.projectpointsFromGridToImage(image23_path,vec,keys,&frame23, "Frame 23", &cntr);
    grid.projectpointsFromGridToImage(image24_path,vec,keys,&frame24, "Frame 24", &cntr);
    grid.projectpointsFromGridToImage(image25_path,vec,keys,&frame25, "Frame 25", &cntr);




//    waitKey(0);

    video();

}


