//
// Created by  Elon Grubman on 14/11/2019.
//

#ifndef NC_RD_FACEMASK_FRAME_H
#define NC_RD_FACEMASK_FRAME_H

#include <nlohmann/json.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>


using namespace std;
using namespace cv;
using json = nlohmann::json;



struct worldPoint{
    long id;
    float x;
    float y;
    float z;
    void print(){
        cout << id << "," << x << "," << y << "," << z << endl;
    }
};

class Frame {
    Matx<float, 4, 4> A;
    Matx<float, 3, 3> K;
    Matx<float, 3, 3> camori_xy;
    Matx<float, 3, 3> camori;
    Matx<float, 3, 1> campos;
    Matx<float, 3, 3> R;
    Matx<float, 3, 1> T;
    Matx<float, 3, 1> camz;

public:
    Frame(vector<float>& C1, vector<float>& C2) {
//        cout << "testA" << endl;
        /// create A matrix
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                A(j, i) = C1[i * 4 + j];
//                cout << A(j,i) << endl;
            }
        }

        /// Create K matrix and camori_xy
//        cout << "testK & camori" << endl;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                K(j, i) = C2[i * 3 + j];
                camori_xy(i,j) = A(i , j);

//                cout << "k :" << K(j,i) << endl;
//                cout << "camori_xy: " << camori_xy(i,j) << endl;
            }
        }

        /// create T_temp
        Matx<float, 3, 3> T_temp(1,0,0,
                                 0,-1,0,
                                 0,0,-1);

        /// create camori
        camori = camori_xy * T_temp;

        /// create R
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R(j,i) = camori(i,j);
            }
        }

        /// create campos ans camz
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 1; j++) {
                campos(i,j) = A(i, 3);
                camz(i,j) = camori(i,2);
            }
        }

        /// create T
        T = -1 * R * campos;
    }

    Point2d projection(Matx<float,3,1> Pw){
        Matx<float , 3, 1> Pc = (R * Pw) + T ;
        float Pd = Pc(2,0);
        Matx<float , 3, 1> Pn = Pc * (1/Pd);
        Matx<float , 3, 1> Pi_ = K * Pn;
        Point2d p(Pi_(0,0), Pi_(1,0));
        return p;
    }

    Matx<float ,3,3> getK(){
        return this->K;
    }
    Matx<float ,4,4> getA(){
        return this->A;
    }
    Matx<float ,3,3> getCamori_xy(){
        return this->camori_xy;
    }
    Matx<float ,3,3> getR(){
        return this->R;
    }
    Matx<float ,3,1> getCampos(){
        return this->campos;
    }
    Matx<float ,3,1> getT(){
        return this->T;
    }
    Matx<float ,3,3> getCamori(){
        return this->camori;
    }
    Matx<float ,3,1> getCamz(){
        return this->camz;
    }
};

/*!
 * Function to parse the Json file of image
 * @param pathToJson - path to the Json file
 * @param c1 - vector to be filled cam_data[0:15]
 * @param c2 vector to be filled cam_data[16:24]
 * @param features - vector to hold all the features
 */
void parseJson(string pathToJson, vector<float>* c1, vector<float>* c2, vector<worldPoint>* features){

    ifstream i(pathToJson);
    json j = json::parse(i);
    string cam_data = j["cam_data"];
    stringstream ss(cam_data);
    float tmp;

    for (int i=0; ss >> tmp;i++) {
        if (i < 16) {
            c1->push_back(tmp);
        }
        else {
            c2->push_back(tmp);
        }
        if (ss.peek() == ',') ss.ignore();
    }

    vector<string> str_features = j["features"];
    for (int k = 0; k < str_features.size(); ++k) {

        stringstream ss2(str_features[k]);
        worldPoint w;
        int counter = 0;
        for (; ss2 >> tmp; counter++) {
            switch (counter) {
                case 0:
                    w.id = tmp;
                    if (ss2.peek() == ',') {
                        ss2.ignore();
                    }
                    break;
                case 1:
                    w.x = tmp;
                    if (ss2.peek() == ',') {
                        ss2.ignore();
                    }
                    break;
                case 2:
                    w.y = tmp;
                    if (ss2.peek() == ',') {
                        ss2.ignore();
                    }
                    break;
                case 3:
                    w.z = tmp;
                    counter = 0;
                    features->push_back(w);
                    if (ss2.peek() == ',') {
                        ss2.ignore();
                    }
            }
        }
    }
}

/*!
 * function that recive point from the world and project her to the image
 * @param path_to_image - the full path to the local place of the image
 * @param i - x coordinate
 * @param j - y coordinate
 * @param k - z coordinate
 * @param frame - pointer to current frame
 * @param window_name - the title to the window image
 */
void projectFromWorldToImage(string path_to_image, float i, float j, float k, Frame* frame, string window_name){
    Mat im1 = imread(path_to_image);
    Matx<float, 3, 1> Pw(i, j, k);
    Point2d p1 = frame->projection(Pw);
    circle(im1, p1, 3, Scalar(0, 0, 255), -1);
    imshow(window_name, im1);
    waitKey(0);
}


#endif //NC_RD_FACEMASK_FRAME_H
