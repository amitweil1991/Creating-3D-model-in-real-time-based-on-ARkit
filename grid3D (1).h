//
// Created by  Elon Grubman on 14/11/2019.
//

#ifndef NC_RD_FACEMASK_GRID3D_H
#define NC_RD_FACEMASK_GRID3D_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <map>
#include <math.h>
#include <cstdlib>
#include <tgmath.h>


#define GRID_SIZE 11
#define EDGE_COORDINATE ( GRID_SIZE - 1 ) / 2
#define ALPHA 0.01
#define SIGMA 5

using namespace std;
using namespace cv;






class straight_line_equation{
public:
    straight_line_equation(const Matx31f &camPose, const Matx31f &point) : point(point) {
        direction_vector = point - camPose;
    };

    const Matx31f &getDirectionVector() const {
        return direction_vector;
    }

    void setDirectionVector(const Matx31f &directionVector) {
        direction_vector = directionVector;
    }

    void setPoint(const Matx31f &point) {
        straight_line_equation::point = point;
    }

    const Matx31f &getPoint() const {
        return point;
    }

private:
    Matx31f direction_vector;
    Matx31f point;
};


class voxel{
    float distance;
    float confidence;
public:
    voxel(float val=9999, float confd=-9999): distance(val), confidence(confd){}
    void setConfidence(float confd){
        confidence = confd;
    }
    void setDistance(float val){
        distance = val;
    }
    float getConfidence(){
        return confidence;
    }
    float getDistance(){
        return distance;
    }
};


class grid3D{
    map<tuple<int,int,int>, voxel> grid;
    float step_size;
    Matx<float, 3, 3> grid_ori;
    Matx<float, 3, 1> grid_center_w;

public:
    grid3D(float step, Frame* creation_frame): step_size(step), grid_ori(creation_frame->getCamori()){


        Matx31f grid_center_c(0,0,2);
        ///calc R_inverse
        Matx<float , 3, 3> R_inverse = creation_frame->getR().inv();
        grid_center_w = R_inverse * (grid_center_c - creation_frame->getT());
    }
    grid3D(float step, Matx33f gridOri, Matx31f grid_center_world): step_size(step), grid_ori(gridOri), grid_center_w(grid_center_world){};



    Matx<float ,3 ,1> getPw(){
        return this->grid_center_w;
    }
    /*!
     * function that create straight line equation
     * @param Pw - the point that the line go through from the camera
     * @param frame - the frame that the camera defines
     * @return the created line (straight line equation)
     */
    straight_line_equation createStraightLineEquation(Matx31f Pw, Frame& frame){
        Matx31f Pc = frame.getR() * Pw + frame.getT();
        Matx31f Pn = Pc * (1/Pc(2,0));
        Matx31f P0 = -1 * this->grid_ori * ( -1 * this->grid_ori.inv() * this->grid_center_w );
        Matx31f direction_vector = this->grid_ori * Pn;
        straight_line_equation created_line (direction_vector, P0);
        return created_line;
    }

    map<tuple<int,int,int>,voxel>& getGrid(){
        return grid;
    }

    /*!
     * function that convert voxel's center coordinates in the camera to its coordinates in the world
     * the just notify, the function gets the voxel's [i][j][k]
     * @param i - row number
     * @param j - col number
     * @param k - depth number
     * @return - its position in the world as Matx
     */
    Matx<float, 3, 1> mapFromGridToWorld(float i, float j, float k){
        Matx31f voxel_coordinates(i, j, k);
        Matx<float, 3, 1> P_voxel_center_w = grid_center_w + step_size * grid_ori * voxel_coordinates;
        return P_voxel_center_w;
    }


    void projectpointsFromGridToImage(string image, vector<Point3d>& voxels_indexes, vector<tuple<int, int, int>>& black_voxels, Frame* frame_to_project, string image_title, int* cntr){
        Mat im1 = imread(image);
        Point2d points[9];
        int tmp=0;

        for (int i = 0; i < voxels_indexes.size(); ++i) {
            Matx<float , 3, 1> P_voxel_center_w = mapFromGridToWorld(voxels_indexes[i].x,voxels_indexes[i].y,voxels_indexes[i].z);

            Point2d p1 = frame_to_project->projection(P_voxel_center_w);
            circle(im1, p1, 6, Scalar(0, 0, 255), -1);
            points[tmp++] = p1;
        }

        for (int i = 0; i < black_voxels.size(); ++i) {
            int x = get<0>(black_voxels[i]);
            int y = get<1>(black_voxels[i]);
            int z = get<2>(black_voxels[i]);

            Matx<float , 3, 1> P_voxel_center_w = mapFromGridToWorld(x,y,z);

            Point2d p1 = frame_to_project->projection(P_voxel_center_w);
            circle(im1, p1, 18, Scalar(0, 0, 0), -1);
        }



        line(im1, points[0], points[1], cv::Scalar(255, 0, 0), 2);
        line(im1, points[0], points[2], cv::Scalar(255, 0, 0), 2);
        line(im1, points[0], points[4], cv::Scalar(255, 0, 0), 2);
        line(im1, points[1], points[3], cv::Scalar(255, 0, 0), 2);
        line(im1, points[1], points[5], cv::Scalar(255, 0, 0), 2);
        line(im1, points[3], points[2], cv::Scalar(255, 0, 0), 2);
        line(im1, points[3], points[7], cv::Scalar(255, 0, 0), 2);
        line(im1, points[2], points[6], cv::Scalar(255, 0, 0), 2);
        line(im1, points[4], points[5], cv::Scalar(255, 0, 0), 2);
        line(im1, points[4], points[6], cv::Scalar(255, 0, 0), 2);
        line(im1, points[7], points[5], cv::Scalar(255, 0, 0), 2);
        line(im1, points[7], points[6], cv::Scalar(255, 0, 0), 2);

        std::string savingName = "/Users/elongrubman/Desktop/outputs/" + std::to_string(*(cntr)) + ".jpg";
        cv::imwrite(savingName, im1);
        *cntr += 1;

//        imshow(image_title, im1);
//        waitKey(0);
    }

    /*!
     * function that returns the coordinates in the grid axis's system for a given voxel
     * @param i - the i index of the voxel
     * @param j - the j index of the voxel
     * @param k - the k index of the voxel
     * @return coordiantes as Matx31f
     */
    Matx31f getCoordinatesFromVoxelIndex(int i, int j, int k){
        Matx31f coordinates(i * step_size, j * step_size, k * step_size);
        return coordinates;
    }

    /*!
     * get the range of the possible slope of a given line intersecting with the grid
     * @param line - the line we're searching its slope's range for intersecting with the grid.
     * @return the range of the slope (lower and upper ranges, represented as a Point2d)
     */
    Point2d getSlopeRange(straight_line_equation line){
        Matx31f positive_boarder_of_gird((EDGE_COORDINATE+0.5) * step_size, (EDGE_COORDINATE+0.5) * step_size, (EDGE_COORDINATE+0.5) * step_size);
        Matx31f negative_boarder_of_gird(-(EDGE_COORDINATE+0.5) * step_size, -(EDGE_COORDINATE+0.5) * step_size, -(EDGE_COORDINATE+0.5) * step_size);
        Matx31f middle_max_value = (positive_boarder_of_gird - line.getPoint());
        Matx31f middle_min_value = (negative_boarder_of_gird - line.getPoint());
        float upper_boarder = 0;
        float lower_boarder = 0;
        vector<float> s_smaller_then;
        vector<float> s_bigger_then;
        vector<float> range_one;
        vector<float> range_two;
        vector<float> range_three;
        /// if x coordinate is zero in the direction vector we cant devide in zero.
        if(line.getDirectionVector()(0,0) == 0) {
            if (line.getDirectionVector()(0, 1) != 0) {
                middle_max_value(0, 1) /= line.getDirectionVector()(0, 1);
                middle_min_value(0, 1) /= line.getDirectionVector()(0, 1);
                range_one.push_back(middle_max_value(0, 1));
                range_one.push_back(middle_min_value(0, 1));
                if (line.getDirectionVector()(0, 1) < 0) {
                    s_bigger_then.push_back(middle_max_value(0, 1));
                    s_smaller_then.push_back(middle_min_value(0, 1));
                } else {
                    s_bigger_then.push_back(middle_min_value(0, 1));
                    s_smaller_then.push_back(middle_max_value(0, 1));
                }
                /// check if the line will never touch the grid
                upper_boarder = findMinInVector(s_smaller_then);
                lower_boarder = findMaxInVector(s_bigger_then);
                range_one.clear();
                range_two.clear();
                return Point2d(lower_boarder, upper_boarder);

            }
            if(line.getDirectionVector()(0, 2) != 0){
                middle_max_value(0,2) /= line.getDirectionVector()(0,2);
                middle_min_value(0,2) /= line.getDirectionVector()(0,2);
                range_two.push_back(middle_max_value(0, 2));
                range_two.push_back(middle_min_value(0, 2));
                if(line.getDirectionVector()(0,2) < 0){
                    s_bigger_then.push_back(middle_max_value(0,2));
                    s_smaller_then.push_back( middle_min_value(0,2));
                }
                else{
                    s_bigger_then.push_back(middle_min_value(0,2));
                    s_smaller_then.push_back( middle_max_value(0,2));
                }
                /// check if the line will never touch the grid
                upper_boarder = findMinInVector(s_smaller_then);
                lower_boarder = findMaxInVector(s_bigger_then);
                range_one.clear();
                range_two.clear();
                return Point2d(lower_boarder, upper_boarder);

             }

            /// check if the line will never touch the grid
            float min_one = findMinInVector(range_one);
            float max_one = findMaxInVector(range_one);
            float min_two = findMinInVector(range_two);
            float max_two = findMaxInVector(range_two);
            if(min_one > max_two || max_one < min_two){
                cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999,-9999);
            }
            upper_boarder = findMinInVector(s_smaller_then);
            lower_boarder = findMaxInVector(s_bigger_then);
            range_one.clear();
            range_two.clear();
            return Point2d(lower_boarder, upper_boarder);

        }
        /// if y coordinate is zero in the direction vector we cant devide in zero.
        if(line.getDirectionVector()(0,1) == 0){
            middle_max_value(0, 0) /= line.getDirectionVector()(0, 0);
            middle_min_value(0, 0) /= line.getDirectionVector()(0, 0);
            range_one.push_back(middle_max_value(0, 0));
            range_one.push_back(middle_min_value(0, 0));
            if (line.getDirectionVector()(0, 0) < 0) {
                s_bigger_then.push_back(middle_max_value(0, 0));
                s_smaller_then.push_back(middle_min_value(0, 0));
            } else {
                s_bigger_then.push_back(middle_min_value(0, 0));
                s_smaller_then.push_back(middle_max_value(0, 0));
            }

            if((line.getDirectionVector()(0,2) != 0)) {
                middle_max_value(0, 2) /= line.getDirectionVector()(0, 2);
                middle_min_value(0, 2) /= line.getDirectionVector()(0, 2);
                range_two.push_back(middle_max_value(0, 2));
                range_two.push_back(middle_min_value(0, 2));
                if (line.getDirectionVector()(0, 2) < 0) {
                    s_bigger_then.push_back(middle_max_value(0, 2));
                    s_smaller_then.push_back(middle_min_value(0, 2));
                } else {
                    s_bigger_then.push_back(middle_min_value(0, 2));
                    s_smaller_then.push_back(middle_max_value(0, 2));
                }
                float min_one = findMinInVector(range_one);
                float max_one = findMaxInVector(range_one);
                float min_two = findMinInVector(range_two);
                float max_two = findMaxInVector(range_two);
                if(min_one > max_two || max_one < min_two){
                    cout << "The line is Out of the Grid" << endl;
                    return Point2d(-9999,-9999);
                }
                upper_boarder = findMinInVector(s_smaller_then);
                lower_boarder = findMaxInVector(s_bigger_then);
                range_one.clear();
                range_two.clear();
                return Point2d(lower_boarder, upper_boarder);
            }

            upper_boarder = findMinInVector(s_smaller_then);
            lower_boarder = findMaxInVector(s_bigger_then);
            range_one.clear();
            range_two.clear();
            return Point2d(lower_boarder, upper_boarder);

        }

        /// if z coordinate is zero in the direction vector we cant devide in zero.
        if(line.getDirectionVector()(0,2) == 0){
            middle_max_value(0,1) /= line.getDirectionVector()(0,1);
            middle_min_value(0,1) /= line.getDirectionVector()(0,1);
            range_one.push_back(middle_max_value(0, 1));
            range_one.push_back(middle_min_value(0, 1));
            if(line.getDirectionVector()(0,1) < 0){
                s_bigger_then.push_back(middle_max_value(0,1));
                s_smaller_then.push_back( middle_min_value(0,1));
            }
            else{
                s_bigger_then.push_back(middle_min_value(0,1));
                s_smaller_then.push_back( middle_max_value(0,1));
            }

            middle_max_value(0,0) /= line.getDirectionVector()(0,0);
            middle_min_value(0,0) /= line.getDirectionVector()(0,0);
            range_two.push_back(middle_max_value(0, 0));
            range_two.push_back(middle_min_value(0, 0));
            if(line.getDirectionVector()(0,0) < 0){
                s_bigger_then.push_back(middle_max_value(0,0));
                s_smaller_then.push_back( middle_min_value(0,0));
            }
            else{
                s_bigger_then.push_back(middle_min_value(0,0));
                s_smaller_then.push_back( middle_max_value(0,0));
            }
            float min_one = findMinInVector(range_one);
            float max_one = findMaxInVector(range_one);
            float min_two = findMinInVector(range_two);
            float max_two = findMaxInVector(range_two);
            if(min_one > max_two || max_one < min_two){
                cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999,-9999);
            }
            upper_boarder = findMinInVector(s_smaller_then);
            lower_boarder = findMaxInVector(s_bigger_then);
            return Point2d(lower_boarder, upper_boarder);

        }
        else {
            middle_max_value(0, 0) /= line.getDirectionVector()(0, 0);
            middle_min_value(0,0) /= line.getDirectionVector()(0,0);
            range_one.push_back(middle_max_value(0, 0));
            range_one.push_back(middle_min_value(0, 0));
            if(line.getDirectionVector()(0,0) < 0){
                s_bigger_then.push_back(middle_max_value(0,0));
                s_smaller_then.push_back( middle_min_value(0,0));
            }
            else{
                s_bigger_then.push_back(middle_min_value(0,0));
                s_smaller_then.push_back( middle_max_value(0,0));
            }
            middle_max_value(0, 1) /= line.getDirectionVector()(0, 1);
            middle_min_value(0,1) /= line.getDirectionVector()(0,1);
            range_two.push_back(middle_max_value(0, 1));
            range_two.push_back(middle_min_value(0, 1));
            if(line.getDirectionVector()(0,1) < 0){
                s_bigger_then.push_back(middle_max_value(0,1));
                s_smaller_then.push_back( middle_min_value(0,1));
            }
            else{
                s_bigger_then.push_back(middle_min_value(0,1));
                s_smaller_then.push_back( middle_max_value(0,1));
            }
            middle_max_value(0, 2) /= line.getDirectionVector()(0, 2);
            middle_min_value(0,2) /= line.getDirectionVector()(0,2);
            range_three.push_back(middle_max_value(0, 2));
            range_three.push_back(middle_min_value(0, 2));
            if(line.getDirectionVector()(0,2) < 0){
                s_bigger_then.push_back(middle_max_value(0,2));
                s_smaller_then.push_back( middle_min_value(0,2));
            }
            else{
                s_bigger_then.push_back(middle_min_value(0,2));
                s_smaller_then.push_back( middle_max_value(0,2));
            }
            float min_one = findMinInVector(range_one);
            float max_one = findMaxInVector(range_one);
            float min_two = findMinInVector(range_two);
            float max_two = findMaxInVector(range_two);
            float min_three = findMinInVector(range_three);
            float max_three = findMaxInVector(range_three);
            if(min_one > max_two || max_one < min_two){
                cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999,-9999);
            }
            if(min_one > max_three || max_one < min_three){
                cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999,-9999);
            }
            if(min_two > max_three || max_three < min_two){
                cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999,-9999);
            }
            upper_boarder = findMinInVector(s_smaller_then);
            lower_boarder = findMaxInVector(s_bigger_then);
            range_one.clear();
            range_two.clear();
            range_three.clear();
        }
        return Point2d(lower_boarder, upper_boarder);
    }

    /*!
     * find minimum in a given vector
     * @param vec - the vector.
     * @return minimum of the vector
     */
    float findMinInVector(vector<float> vec){
        float min = vec[0];
        for(int i = 0; i < vec.size(); i++){
            if(vec[i] < min){
                min = vec[i];
            }
        }
        return min;
    }
    /*!
    * find maximum in a given vector
    * @param vec - the vector.
    * @return maximum of the vector
    */
    float findMaxInVector(vector<float> vec){
        float max = vec[0];
        for(int i = 0; i < vec.size(); i++){
            if(vec[i] > max){
                max = vec[i];
            }
        }
        return max;
    }
    /*!
     * function for getting the intersection points (entrance, and exit) of a given line with the grid, by its range of slope
     * @param line  the line we're intersecting with
     * @param S - the range of the slope
     * @return - tuple of vectors consisnting the entrance and the exit points.
     */
    tuple<Matx31f, Matx31f> findIntersectionPoint(straight_line_equation line, Point2d S){
        Matx31f entrancePoint = line.getPoint() + S.x * line.getDirectionVector();
        Matx31f exitPoint = line.getPoint() + S.y * line.getDirectionVector();
        tuple<Matx31f, Matx31f> tup(entrancePoint,exitPoint);
        return tup;
    }
    /*!
     * this function gets coordinates in the grid axis system, and returns the indexes of the voxel from the memory(the key of the map),
     * that satisfies that the coordinates fall on its area, if no voxel is found we will push one to the grid, with
     * suitable confidence and distance (the grid contains only voxels that are colored by a ray, and the point is certainly in at leat one voxel area).
       if the coordinates fall in the area of two voxel (if one of the coordinates
     * is exactly step / 2 , we will return both of them), thats why we return a vector.
     * @param i - the x coordinates
     * @param j - the y coordinates
     * @param k - the z coordinates
     * @param line - the line that are now coloring the grid


     * @return - the voxels indexes as a vecotr of tuples.
     */
    vector<tuple<int, int, int>> getVoxelFromCoordinatesOrPush(float i, float j, float k, straight_line_equation line) {
        /// in case the point is out of the grid
        vector<tuple<int,int,int>> keys;
        if(fabs(i) > (EDGE_COORDINATE + 0.5) * step_size || fabs(j) > (EDGE_COORDINATE + 0.5) * step_size || fabs(k) > (EDGE_COORDINATE + 0.5) * step_size){
            cout << "coordinates is out of the grid's bound" << endl;
            return keys;
        }

        float x = i / step_size;
        float y = j / step_size;
        float z = k / step_size;

        int floor_x, floor_y, floor_z, upper_x, upper_y, upper_z = 0;
        bool flag_x, flag_y, flag_z = false; /// this flags denothing wheter the coordinate it in between two voxels
        if (fabs(x - (int) x) == 0.5) {
            floor_x = (int) x;
            if(x >= 0) upper_x = (int) x + 1;
            else upper_x = (int)x - 1;
            flag_x = true;
        }
        if (fabs(y - (int) y) == 0.5) {
            floor_y = (int) y;
            if(y >= 0) upper_y = (int) y + 1;
            else upper_y = (int)y - 1;
            flag_y = true;
        }
        if (fabs(z - (int) z )== 0.5) {
            floor_z = (int) z;
            if(z >= 0) upper_z = (int) z + 1;
            else upper_z = (int)z - 1;
            flag_z = true;
        }
        if (abs(x - (int) x) < 0.5)
            x = (int) x;
        else if (fabs(x - (int) x > 0.5)){
            if(x>=0) x = (int) x + 1;
            else x = (int)x - 1;
        }

        if (fabs(y - (int) y) < 0.5)
            y = (int) y;
        else if (fabs(y - (int) y > 0.5)){
            if(y >= 0) y = (int) y + 1;
            else y = (int)y - 1;
        }

        if (fabs(z - (int) z) < 0.5)
            z = (int) z;
        else if (fabs(z - (int) z > 0.5)){
            if(z >= 0) z = (int) z + 1;
            else z = (int)z - 1;
        }

//        vector<tuple<int, int, int>> keys;
        if (flag_x == true && flag_y == false && flag_z == false) {

            if (fabs(upper_x) > EDGE_COORDINATE) {
                tuple<int, int, int> key(floor_x, y, z);
                keys.push_back(key);
            } else if (fabs(floor_x) > EDGE_COORDINATE) {
                tuple<int, int, int> key(upper_x, y, z);
                keys.push_back(key);
            } else {
                tuple<int, int, int> key1(floor_x, y, z);
                tuple<int, int, int> key2(upper_x, y, z);
                keys.push_back(key1);
                keys.push_back(key2);
            }

        } else if (flag_x == false && flag_y == true && flag_z == false) {

            if (fabs(upper_y) > EDGE_COORDINATE) {
                tuple<int, int, int> key(x, floor_y, z);
                keys.push_back(key);
            } else if (fabs(floor_y) > EDGE_COORDINATE) {
                tuple<int, int, int> key(x, upper_y, z);
                keys.push_back(key);
            } else {
                tuple<int, int, int> key1(x, floor_y, z);
                tuple<int, int, int> key2(x, upper_y, z);
                keys.push_back(key1);
                keys.push_back(key2);
            }
        } else if (flag_x == false && flag_y == false && flag_z == true) {

            if (fabs(upper_z) > EDGE_COORDINATE) {
                tuple<int, int, int> key(x, y, floor_z);
                keys.push_back(key);
            } else if (fabs(floor_z) > EDGE_COORDINATE) {
                tuple<int, int, int> key(x, y, upper_z);
                keys.push_back(key);
            } else {
                tuple<int, int, int> key1(x, y, floor_z);
                tuple<int, int, int> key2(x, y, upper_z);
                keys.push_back(key1);
                keys.push_back(key2);
            }
        } else if (flag_x == true && flag_y == true && flag_z == false) {

            if (fabs(upper_x) > EDGE_COORDINATE) {
                if (fabs(upper_y) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(floor_x, floor_y, z);
                    keys.push_back(key);
                } else if (fabs(floor_y) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(floor_x, upper_y, z);
                    keys.push_back(key);
                } else {
                    tuple<int, int, int> key1(floor_x, floor_y, z);
                    tuple<int, int, int> key2(floor_x, upper_y, z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                }
            } else if (fabs(floor_x) > EDGE_COORDINATE) {
                if (fabs(upper_y) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(upper_x, floor_y, z);
                    keys.push_back(key);
                } else if (fabs(floor_y) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(upper_x, upper_y, z);
                    keys.push_back(key);
                } else {
                    tuple<int, int, int> key1(upper_x, floor_y, z);
                    tuple<int, int, int> key2(upper_x, upper_y, z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                }
            } else {
                if (fabs(upper_y) > EDGE_COORDINATE) {
                    tuple<int, int, int> key1(floor_x, floor_y, z);
                    tuple<int, int, int> key2(upper_x, floor_y, z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                } else if (fabs(floor_y) > EDGE_COORDINATE) {
                    tuple<int, int, int> key1(floor_x, upper_y, z);
                    tuple<int, int, int> key2(upper_x, upper_y, z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                } else {
                    tuple<int, int, int> key1(floor_x, floor_y, z);
                    tuple<int, int, int> key2(floor_x, upper_y, z);
                    tuple<int, int, int> key3(upper_x, floor_y, z);
                    tuple<int, int, int> key4(upper_x, upper_y, z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                    keys.push_back(key3);
                    keys.push_back(key4);
                }
            }
        } else if (flag_x == true && flag_y == false && flag_z == true) {

            if (fabs(upper_x) > EDGE_COORDINATE) {
                if (fabs(upper_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(floor_x, y, floor_z);
                    keys.push_back(key);
                } else if (fabs(floor_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(floor_x, y, upper_z);
                    keys.push_back(key);
                } else {
                    tuple<int, int, int> key1(floor_x, y, floor_z);
                    tuple<int, int, int> key2(floor_x, y, upper_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                }
            } else if (fabs(floor_x) > EDGE_COORDINATE) {
                if (fabs(upper_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(upper_x, y, upper_z);
                    keys.push_back(key);
                } else if (fabs(floor_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(upper_x, y, upper_z);
                    keys.push_back(key);
                } else {
                    tuple<int, int, int> key1(upper_x, y, floor_z);
                    tuple<int, int, int> key2(upper_x, y, upper_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                }
            } else {
                if (fabs(upper_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key1(floor_x, y, floor_z);
                    tuple<int, int, int> key2(upper_x, y, floor_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                } else if (fabs(floor_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key1(floor_x, y, upper_z);
                    tuple<int, int, int> key2(upper_x, y, upper_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                } else {
                    tuple<int, int, int> key1(floor_x, y, floor_z);
                    tuple<int, int, int> key2(floor_x, y, upper_z);
                    tuple<int, int, int> key3(upper_x, y, floor_z);
                    tuple<int, int, int> key4(upper_x, y, upper_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                    keys.push_back(key3);
                    keys.push_back(key4);
                }
            }
        } else if (flag_x == false && flag_y == true && flag_z == true) {
            if (fabs(upper_y) > EDGE_COORDINATE) {
                if (fabs(upper_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(x, floor_y, floor_z);
                    keys.push_back(key);
                } else if (fabs(floor_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(x, floor_y, upper_z);
                    keys.push_back(key);
                } else {
                    tuple<int, int, int> key1(x, floor_y, floor_z);
                    tuple<int, int, int> key2(x, floor_y, upper_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                }
            } else if (fabs(floor_y) > EDGE_COORDINATE) {
                if (fabs(upper_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(x, upper_y, upper_z);
                    keys.push_back(key);
                } else if (fabs(floor_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key(x, upper_y, upper_z);
                    keys.push_back(key);
                } else {
                    tuple<int, int, int> key1(x, upper_y, floor_z);
                    tuple<int, int, int> key2(x, upper_y, upper_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                }
            } else {
                if (fabs(upper_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key1(x, floor_y, floor_z);
                    tuple<int, int, int> key2(x, upper_y, floor_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                } else if (fabs(floor_z) > EDGE_COORDINATE) {
                    tuple<int, int, int> key1(x, floor_y, upper_z);
                    tuple<int, int, int> key2(x, upper_y, upper_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                } else {
                    tuple<int, int, int> key1(x, floor_y, floor_z);
                    tuple<int, int, int> key2(x, floor_y, upper_z);
                    tuple<int, int, int> key3(x, upper_y, floor_z);
                    tuple<int, int, int> key4(x, upper_y, upper_z);
                    keys.push_back(key1);
                    keys.push_back(key2);
                    keys.push_back(key3);
                    keys.push_back(key4);
                }
            }
        } else if (flag_x == true && flag_y == true && flag_z == true) {
            if (fabs(upper_x) > EDGE_COORDINATE) {
                if (fabs(upper_y) > EDGE_COORDINATE) {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key(floor_x, floor_y, floor_z);
                        keys.push_back(key);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key(floor_x, floor_y, upper_z);
                        keys.push_back(key);
                    } else {
                        tuple<int, int, int> key1(floor_x, floor_y, floor_z);
                        tuple<int, int, int> key2(floor_x, floor_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    }
                } else if (fabs(floor_y) > EDGE_COORDINATE) {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key(floor_x, upper_y, upper_z);
                        keys.push_back(key);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key(floor_x, upper_y, upper_z);
                        keys.push_back(key);
                    } else {
                        tuple<int, int, int> key1(floor_x, upper_y, floor_z);
                        tuple<int, int, int> key2(floor_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    }
                } else {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(floor_x, floor_y, floor_z);
                        tuple<int, int, int> key2(floor_x, upper_y, floor_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(floor_x, floor_y, upper_z);
                        tuple<int, int, int> key2(floor_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    } else {
                        tuple<int, int, int> key1(floor_x, floor_y, floor_z);
                        tuple<int, int, int> key2(floor_x, upper_y, floor_z);
                        tuple<int, int, int> key3(floor_x, floor_y, upper_z);
                        tuple<int, int, int> key4(floor_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                        keys.push_back(key3);
                        keys.push_back(key4);
                    }
                }
            }
            else if (fabs(floor_x) > EDGE_COORDINATE) {
                if (fabs(upper_y) > EDGE_COORDINATE) {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key(upper_x, floor_y, floor_z);
                        keys.push_back(key);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key(upper_x, floor_y, upper_z);
                        keys.push_back(key);
                    } else {
                        tuple<int, int, int> key1(upper_x, floor_y, floor_z);
                        tuple<int, int, int> key2(upper_x, floor_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    }
                } else if (fabs(floor_y) > EDGE_COORDINATE) {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key(upper_x, upper_y, upper_z);
                        keys.push_back(key);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key(upper_x, upper_y, upper_z);
                        keys.push_back(key);
                    } else {
                        tuple<int, int, int> key1(upper_x, upper_y, floor_z);
                        tuple<int, int, int> key2(upper_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    }
                } else {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(upper_x, floor_y, floor_z);
                        tuple<int, int, int> key2(upper_x, upper_y, floor_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(upper_x, floor_y, upper_z);
                        tuple<int, int, int> key2(upper_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    } else {
                        tuple<int, int, int> key1(upper_x, floor_y, floor_z);
                        tuple<int, int, int> key2(upper_x, upper_y, floor_z);
                        tuple<int, int, int> key3(upper_x, floor_y, upper_z);
                        tuple<int, int, int> key4(upper_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                        keys.push_back(key3);
                        keys.push_back(key4);
                    }
                }
            }
            else {
                if (fabs(upper_y) > EDGE_COORDINATE) {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(upper_x, floor_y, floor_z);
                        tuple<int, int, int> key2(floor_x, floor_y, floor_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(upper_x, floor_y, upper_z);
                        tuple<int, int, int> key2(floor_x, floor_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    } else {
                        tuple<int, int, int> key1(upper_x, floor_y, floor_z);
                        tuple<int, int, int> key2(upper_x, floor_y, upper_z);
                        tuple<int, int, int> key3(floor_x, floor_y, floor_z);
                        tuple<int, int, int> key4(floor_x, floor_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                        keys.push_back(key3);
                        keys.push_back(key4);
                    }
                } else if (fabs(floor_y) > EDGE_COORDINATE) {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(upper_x, upper_y, floor_z);
                        tuple<int, int, int> key2(floor_x, upper_y, floor_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(upper_x, upper_y, upper_z);
                        tuple<int, int, int> key2(floor_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                    } else {
                        tuple<int, int, int> key1(upper_x, upper_y, floor_z);
                        tuple<int, int, int> key2(upper_x, upper_y, upper_z);
                        tuple<int, int, int> key3(floor_x, upper_y, floor_z);
                        tuple<int, int, int> key4(floor_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                        keys.push_back(key3);
                        keys.push_back(key4);
                    }
                } else {
                    if (fabs(upper_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(upper_x, floor_y, floor_z);
                        tuple<int, int, int> key2(upper_x, upper_y, floor_z);
                        tuple<int, int, int> key3(floor_x, floor_y, floor_z);
                        tuple<int, int, int> key4(floor_x, upper_y, floor_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                        keys.push_back(key3);
                        keys.push_back(key4);
                    } else if (fabs(floor_z) > EDGE_COORDINATE) {
                        tuple<int, int, int> key1(upper_x, floor_y, upper_z);
                        tuple<int, int, int> key2(upper_x, upper_y, upper_z);
                        tuple<int, int, int> key3(floor_x, floor_y, upper_z);
                        tuple<int, int, int> key4(floor_x, upper_y, upper_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                        keys.push_back(key3);
                        keys.push_back(key4);
                    } else {
                        tuple<int, int, int> key1(upper_x, floor_y, upper_z);
                        tuple<int, int, int> key2(upper_x, upper_y, upper_z);
                        tuple<int, int, int> key3(floor_x, floor_y, upper_z);
                        tuple<int, int, int> key4(floor_x, upper_y, upper_z);
                        tuple<int, int, int> key5(upper_x, floor_y, floor_z);
                        tuple<int, int, int> key6(upper_x, upper_y, floor_z);
                        tuple<int, int, int> key7(floor_x, floor_y, floor_z);
                        tuple<int, int, int> key8(floor_x, upper_y, floor_z);
                        keys.push_back(key1);
                        keys.push_back(key2);
                        keys.push_back(key3);
                        keys.push_back(key4);
                        keys.push_back(key5);
                        keys.push_back(key6);
                        keys.push_back(key7);
                        keys.push_back(key8);
                    }
                }
            }
        }
        else {
            tuple<int, int, int> key(x, y, z);
            keys.push_back(key);
        }
        UpdateVoxelsFromIndexes(keys, line);
        return keys;

    }

    /*!
     * this function is responsible for conducting the bresenham algorithim in order to color the
     * relevant voxels, the voxels that the ray go trough them.
     * @param line - the equation of the ray
     * @param entrance_voxel - the first voxel that the ray touches when it enter the grid range.
     */
    void bresenhamAlgorithim(straight_line_equation line, tuple<int,int,int> entrance_voxel) {
        float direction_x = line.getDirectionVector()(0);
        float direction_y = line.getDirectionVector()(1);
        float direction_z = line.getDirectionVector()(2);
        float slope = 0;
        float dominant_direction = max(fabs(direction_x), max(fabs(direction_y), fabs(direction_z)));
        float next_point_x = 0;
        float next_point_y = 0;
        float next_point_z = 0;
        /// case the dominant direction is x
        tuple<int, int, int> next_voxel = entrance_voxel;
        vector<tuple<int, int, int>> keys;
        while (true) {
            if (dominant_direction == fabs(direction_x)) {
                get<0>(next_voxel) += (direction_x >= 0) ? 1 : -1 ;
                /// in case we got to a voxel which is out of the grid we've finished.
                next_point_x = get<0>(next_voxel) * step_size;
                /// finding S
                slope = next_point_x - line.getPoint()(0);
                /// finding next y and next z
                next_point_y = line.getPoint()(1) + slope * direction_y;
                next_point_z = line.getPoint()(2) + slope * direction_z;
                /// in case this vector is empty it means that the next voxel is out of the grid and we should break
                auto vec = getVoxelFromCoordinatesOrPush(next_point_x, next_point_y, next_point_z, line);
                if(vec.empty()){
                    break;
                }

            }
            /// case the dominant direction is y
            else if (dominant_direction == fabs(direction_y)) {
                get<1>(next_voxel) += direction_y >= 0 ? 1 : -1;
                /// in case we got to a voxel which is out of the grid we've finished.
                next_point_y = get<1>(next_voxel) * step_size;
                /// finding S
                slope = next_point_y - line.getPoint()(1);
                /// finding next y and next z
                next_point_x = line.getPoint()(0) + slope * direction_x;
                next_point_z = line.getPoint()(2) + slope * direction_z;
                /// in case this vector is empty it means that the next voxel is out of the grid and we should break
                auto vec = getVoxelFromCoordinatesOrPush(next_point_x, next_point_y, next_point_z, line);
                if(vec.empty()){
                    break;
                }
            }
            /// case the dominant direction is z
            else if (dominant_direction == fabs(direction_z)) {
                get<2>(next_voxel) += direction_z >= 0 ? 1 : -1;;
                /// in case we got to a voxel which is out of the grid we've finished.
                next_point_z = get<2>(next_voxel) * step_size;
                /// finding S
                slope = next_point_z - line.getPoint()(2);
                /// finding next y and next z
                next_point_x = line.getPoint()(0) + slope * direction_x;
                next_point_y = line.getPoint()(1) + slope * direction_y;
                auto vec = getVoxelFromCoordinatesOrPush(next_point_x, next_point_y, next_point_z, line);
                /// in case this vector is empty it means that the next voxel is out of the grid and we should break
                if(vec.empty()){
                    break;
                }
            }
        }
    }


    /*!
     * function that get vector of indexes and update the matching voxel distance and confidence,
     * if the voxel hasent been painted yet, we will create one and insert it to the grid.
     * @param keys - vector<tuple>, the indexes
     * @param line - the line that are coloring the the voxel we want to update.
     */
    void UpdateVoxelsFromIndexes(vector<tuple<int,int,int>>& keys, straight_line_equation line) {
        for (int i = 0; i < keys.size(); ++i) {
            auto it = grid.find(keys[i]);
            Matx31f point = getCoordinatesFromVoxelIndex(get<0>(keys[i]),get<1>(keys[i]),get<2>(keys[i]));
            Matx31f direction_vector = line.getDirectionVector();
            float s_voxel = (point(0, 0) - line.getPoint()(0, 0)) / direction_vector(0,0);
            /// in case the voxel does not exist, we create it and insert it into the gird, then returning it.
            if (it == grid.end()) {
                voxel inserted_voxel;
                float confidence = calc_confidence(s_voxel, 1, inserted_voxel);
                float distance = calc_distance(s_voxel, 1, inserted_voxel);
                inserted_voxel.setConfidence(confidence);
                inserted_voxel.setDistance(distance);
                grid.insert(pair<tuple<int, int, int>, voxel>(keys[i], inserted_voxel));
            }
            // in case the voxel have already been colored once, we just update the confidence and the distance
            else{
                float confidence = calc_confidence(s_voxel, 1, get<1>(*it));
                float distance = calc_distance(s_voxel, 1, get<1>(*it));
                get<1>(*it).setConfidence(confidence);
                get<1>(*it).setDistance(distance);
            }
        }
    }


    /*!
 * function for calculating the confidence of a given voxel,
 * the calculation is diffrent if it already has a confidence or it dosent.
 * @param s_voxel - the scalar that if we will multiply the direction vector by him, we will get to the voxel.
 * @param s_point - the scalar that if we will multiply the direction vector by him, we will get to the point that
 * @param vox - the voxel we're calculating the distance
 * @return - the confidence of the voxel.
 */
    float calc_confidence(float s_voxel, float s_point, voxel vox){
        float new_confidence = exp(-(pow(s_voxel - s_point, 2) / (SIGMA * SIGMA)));
        if(vox.getConfidence() == -9999){
            return new_confidence;
        }
        else{
            return (new_confidence + vox.getConfidence()) / 2;
        }
    }



    /*!
     * function for calculating the distance between a voxel and a point that defines a ray,
     * the calculation is different if the voxel already has a distance.
     * @param s_voxel - the scalar that if we will multiply the direction vector by him, we will get to the voxel.
     * @param s_point - the scalar that if we will multiply the direction vector by him, we will get to the point that
     * defines the ray (with the camera).
     * @param vox - the voxel we're calculating the distance
     * @return - the distance between them.
     */
    float calc_distance(float s_voxel, float s_point, voxel vox){
        if(vox.getDistance() == 9999){
            return tanh(ALPHA * (s_voxel - s_point));
        }
        else{
            float new_confidence = calc_confidence(s_voxel, s_point, vox);
            float new_distance = vox.getDistance() * vox.getConfidence() + tanh(ALPHA * (s_voxel - s_point)) * new_confidence;
            return new_distance;
        }
    }

};


/*!
 * function to create the video from all the images
 */
void video ()
{
    VideoWriter out_capture("/Users/elongrubman/Desktop/video.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 3,
                            Size(1920, 1440));
    for (int i = 0; i < 25; i++) {

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

void testProjection(vector<worldPoint> image1_first_features, vector<string> imagesPath,vector<Frame> frames, int* cntr) {
    int num = 102;
    for (int i = 0; i < 12; ++i) {
        Mat im1 = imread(imagesPath[i]);
        Matx<float_t, 3, 1> Pw(image1_first_features[num].x, image1_first_features[num].y,
                               image1_first_features[num].z);
        Point2d p1 = frames[i].projection(Pw);
        circle(im1, p1, 6, Scalar(0, 0, 255), -1);
        std::string savingName = "/Users/elongrubman/Desktop/outputs2/" + std::to_string(*(cntr)) + ".jpg";
        cv::imwrite(savingName, im1);
        *cntr += 1;
    }
}
/*!
 * function for converting worldPoint type to Matx31f type
 * @param point - one of the features
 * @return feature coordinate represent by Matx31f
 */
Matx31f convertWorldPointToMatx(worldPoint point){
    Matx31f p(point.x, point.y,point.z);
    return p;
}
/*!
 * function that get all the Fram's features and return the AVG point for them
 * @param features_point
 * @return AVG point from all the features
 */
Matx31f calc_avg_grid_center(vector<worldPoint> features_point){
    Matx31f avg_point(0,0,0);
    for (int i = 0; i < features_point.size(); ++i) {

        avg_point += convertWorldPointToMatx(features_point[i]);
    }
    avg_point(0,0) /= features_point.size();
    avg_point(1,0) /= features_point.size();
    avg_point(2,0) /= features_point.size();

    return avg_point;
}



#endif //NC_RD_FACEMASK_GRID3D_H
