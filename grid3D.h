//
// Created by  Elon Grubman on 14/11/2019.
//

#ifndef NC_RD_FACEMASK_GRID3D_H
#define NC_RD_FACEMASK_GRID3D_H

#include "utilities.h"

   // std::ofstream


#define GRID_SIZE 201
#define EDGE_COORDINATE ( GRID_SIZE - 1 ) / 2
#define ALPHA 0.01
#define SIGMA 0.03
#define THRESHOLD_CONFIDENCE 0.125
#define MAX_CONFIDENCE 8
#define MAX_DISTANCE 10
#define MIN_DISTANCE -10
#define NORMALIZE_DISTANCE_FACTOR 0.000005
#define FEATURE_CONFIDENCE_BONUS 100

using namespace std;
using namespace cv;


class grid3D;
class voxel;

class GRIDCELL {
public:
    Matx31f p[8];		//position of each corner of the grid in world space
    float val[8];
    GRIDCELL() = default;
    GRIDCELL(vector<tuple<int,int,int>>& indexes_of_vertexes, map<tuple<int,int,int>, voxel>& map, grid3D* grid);

    void print(){
        for(int i = 0; i < 8; i++){
            cout << val[i] << " , ";
        }
        cout << endl;
    }
};






class straight_line_equation{
public:
    straight_line_equation(const Matx31f &camPose, const Matx31f &given_point) : point(camPose) {
        direction_vector = given_point - camPose;
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
    int frame_number;
    bool is_feature;

public:
    voxel(float val = 9999, float confd = -9999, int frameNumber=-1): distance(val), confidence(confd), frame_number(frameNumber), is_feature(false){}
    void setConfidence(float confd){
        confidence = confd;
    }

    int getFrameNumber(){
        return frame_number;
    }
    bool getIsFeature(){
        return is_feature;
    }
    void setFrameNumber(int frameNumber) {
        frame_number = frameNumber;
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
    void setIsFeature(){
        is_feature = true;
    }
};



class grid3D {
    map<tuple<int, int, int>, voxel> grid;
    map<tuple<int, int, int>, voxel> densified_grid;
    map<tuple<int, int, int>, GRIDCELL> grid_cell_map;
    float step_size;
    Matx<float, 3, 3> grid_ori;
    Matx<float, 3, 1> grid_center_w;
    Matx31f grid_center_c;
    Matx33f R;
    Matx31f T;

public:
    grid3D(){
        step_size = 1;
    }
    grid3D(float step, Matx33f gridOri, Matx31f grid_center_world, Matx33f R_frame, Matx31f T_frame) :
            step_size(step), grid_ori(gridOri), grid_center_w(grid_center_world), R(R_frame), T(T_frame) {
        grid_center_c = R * grid_center_w + T;
    };


    Matx<float, 3, 1> getPw() {
        return this->grid_center_w;
    }

    /*!
     * function that create straight line equation
     * @param Pw - the point that the line go through from the camera
     * @param frame - the frame that the camera defines
     * @return the created line (straight line equation)
     */
    straight_line_equation createStraightLineEquation(Matx31f Pw, Frame &frame) {
        Matx31f Pc = frame.getR() * Pw + frame.getT();
        Matx31f Pn = Pc * (1 / Pc(2, 0));
        Matx31f P0 = -1 * this->grid_ori * (-1 * this->grid_ori.inv() * this->grid_center_w);
        Matx31f direction_vector = this->grid_ori * Pn;
        straight_line_equation created_line(direction_vector, P0);
        return created_line;
    }

    map<tuple<int, int, int>, voxel> &getGrid() {
        return grid;
    }

    map<tuple<int, int, int>, voxel> &getDensifiedGrid() {
        return densified_grid;
    }

    map<tuple<int, int, int>, GRIDCELL> &get_grid_cell_map() {
        return grid_cell_map;
    };


    /*!
   * function that convert point from the world, to its grid coordinate system
     * first we move it from world to the axis of the frame the grid was created, and then we calculate grid_center_c - the point
     * according to P_grid = grid_centeter_c - R_creation_frame * Pw + T_creation_frame
   * @param point_world - the point in the world we want to convert
   * @return - its coordinates in the gird as Matx
   */
    Matx<float, 3, 1> mapFromWorldToGrid(Matx31f point_world) {
        Matx31f point_grid = (R * point_world + T) - grid_center_c;
        return point_grid;
    }


    /*!
     * function that convert voxel's center in the camera to its coordinates in the world
     *  just to notify, the function gets the voxel's INDEXES and not its COORDINATES [i][j][k],
     * @param i - row number
     * @param j - col number
     * @param k - depth number
     * @return - its position in the world as Matx
     */
    Matx<float, 3, 1> mapFromGridToWorld(float i, float j, float k) {
        Matx31f voxel_coordinates(i, j, k);
        Matx<float, 3, 1> P_voxel_center_w = grid_center_w + step_size * grid_ori * voxel_coordinates;
        return P_voxel_center_w;
    }

    /*!
     * function that generate color to frame's ray
     * @param frame_number - the number of specific frame
     * @return Scalar function that present the color by RGB
     */
    Scalar makeColorfull(int frame_number) {
        if (frame_number % 3 == 0) return Scalar(frame_number * 10, 0, 0);
        else if (frame_number % 3 == 1) return Scalar(0, frame_number * 10, 0);
        else return Scalar(0, 0, frame_number * 10);
    }

    Scalar convertConfidenceToColor(int confidence) {
        return Scalar(255 - 255 * confidence, 255 - 2 * 255 * abs(confidence - 0.5), confidence * 255);
    }


    /*!
     * this function is responsible to project all the grid points into a given frame, (it will project the grid itself
     * as well, and draw the grid in lines)
     * @param image - the image that are assosiated with the frame
     * @param grid_boundries - the boundries of the grid(its voxels)
     * @param black_voxels - the voxels of the grid (The ones that we're blacked by some ray)
     * @param frame_to_project - the frame we project to
     * @param image_title - the titile of the image.
     * @param cntr - helper value, just of us to create diffrent output images when we call the function for diffrent frames.
     */
    void projectpointsFromGridToImage(string image, vector<Point3d> &grid_boundries,
                                      map<tuple<int, int, int>, voxel> &black_voxels, Frame *frame_to_project,
                                      string image_title, int *cntr) {
        Mat im1 = imread(image);
        Point2d points[9];
        int tmp = 0;

        for (int i = 0; i < grid_boundries.size(); ++i) {
            Matx<float, 3, 1> P_voxel_center_w = mapFromGridToWorld(grid_boundries[i].x, grid_boundries[i].y,
                                                                    grid_boundries[i].z);

            Point2d p1 = frame_to_project->projection(P_voxel_center_w);
            circle(im1, p1, 6, Scalar(0, 0, 255), -1);
            points[tmp++] = p1;
        }

        for (auto it = black_voxels.begin(); it != black_voxels.end(); ++it) {
            int x = get<0>(it->first);
            int y = get<1>(it->first);
            int z = get<2>(it->first);

            int frame_number = it->second.getFrameNumber();

            Matx<float, 3, 1> P_voxel_center_w = mapFromGridToWorld(x, y, z);

            Point2d p1 = frame_to_project->projection(P_voxel_center_w);


            circle(im1, p1, 10, makeColorfull(frame_number), -1);
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
 * function that gets a voxel and calculates a radiues according to the confidence of the voxel
 * just in order that when we present it the voxels with the higher confidence will have bigger circles.
 * @param vox - the voels we're calculating the radius for
 * @return - the radius as an int
 */
    float calcRadiusFromConfidence(voxel &vox) {
        return vox.getConfidence();
    }

    void createConfidenceHeatMap(string image, vector<Point3d> &grid_boundries,
                                 map<tuple<int, int, int>, voxel> &black_voxels, Frame *frame_to_project,
                                 string image_title, int *cntr) {
        Mat im1 = imread(image);
        Point2d points[9];
        int tmp = 0;

        for (int i = 0; i < grid_boundries.size(); ++i) {
            Matx<float, 3, 1> P_voxel_center_w = mapFromGridToWorld(grid_boundries[i].x, grid_boundries[i].y,
                                                                    grid_boundries[i].z);

            Point2d p1 = frame_to_project->projection(P_voxel_center_w);
            circle(im1, p1, 6, Scalar(0, 255, 0), -1);
            points[tmp++] = p1;
        }

        for (auto it = black_voxels.begin(); it != black_voxels.end(); ++it) {
            int x = get<0>(it->first);
            int y = get<1>(it->first);
            int z = get<2>(it->first);
            float radius = calcRadiusFromConfidence(it->second);
            Matx<float, 3, 1> P_voxel_center_w = mapFromGridToWorld(x, y, z);
            Point2d p1 = frame_to_project->projection(P_voxel_center_w);
//            cout << radius << endl;
            if (it->second.getConfidence() >= THRESHOLD_CONFIDENCE) {
//                if(radius < 0.07)  circle(im1, p1, 6, convertConfidenceToColor(it->second.getConfidence()), -1);
//                else circle(im1, p1, radius*8, convertConfidenceToColor(it->second.getConfidence()), -1);
                circle(im1, p1, 7, convertConfidenceToColor(it->second.getConfidence()), -1);
            }
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

        std::string savingName ="output_images/" +  std::to_string(*(cntr)) + ".jpg";
        cv::imwrite(savingName, im1);
        *cntr += 1;
        cout << "finished_heat_map" << endl;
    }


    /*!
     * get the range of the possible slope of a given line intersecting with the grid
     * @param line - the line we're searching its slope's range for intersecting with the grid.
     * @return the range of the slope (lower and upper ranges, represented as a Point2d)
     */


    Point2d getSlopeRange(straight_line_equation line) {
        Matx31f positive_boarder_of_gird((EDGE_COORDINATE + 0.5) * step_size, (EDGE_COORDINATE + 0.5) * step_size,
                                         (EDGE_COORDINATE + 0.5) * step_size);
        Matx31f negative_boarder_of_gird(-(EDGE_COORDINATE + 0.5) * step_size, -(EDGE_COORDINATE + 0.5) * step_size,
                                         -(EDGE_COORDINATE + 0.5) * step_size);
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
        if (line.getDirectionVector()(0, 0) == 0) {
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

            }
            if (line.getDirectionVector()(0, 2) != 0) {
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
                upper_boarder = findMinInVector(s_smaller_then);
                lower_boarder = findMaxInVector(s_bigger_then);


            }

            /// check if the line will never touch the grid
            upper_boarder = findMinInVector(s_smaller_then);
            lower_boarder = findMaxInVector(s_bigger_then);
            if (upper_boarder < lower_boarder) {
//                cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999, -9999);

            }
            return Point2d(lower_boarder, upper_boarder);



        }
        /// if y coordinate is zero in the direction vector we cant devide in zero.
        if (line.getDirectionVector()(0, 1) == 0) {
            /// calculate s range in x
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

            if ((line.getDirectionVector()(0, 2) != 0)) {
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
            }

            upper_boarder = findMinInVector(s_smaller_then);
            lower_boarder = findMaxInVector(s_bigger_then);
            if (upper_boarder < lower_boarder) {
//                cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999, -9999);
            }
            return Point2d(lower_boarder, upper_boarder);

        }

        /// if z coordinate is zero in the direction vector we cant devide in zero.
        if (line.getDirectionVector()(0, 2) == 0) {
            /// calculate s range in y
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
            /// calculate s range in z
            middle_max_value(0, 0) /= line.getDirectionVector()(0, 0);
            middle_min_value(0, 0) /= line.getDirectionVector()(0, 0);
            range_two.push_back(middle_max_value(0, 0));
            range_two.push_back(middle_min_value(0, 0));
            if (line.getDirectionVector()(0, 0) < 0) {
                s_bigger_then.push_back(middle_max_value(0, 0));
                s_smaller_then.push_back(middle_min_value(0, 0));
            } else {
                s_bigger_then.push_back(middle_min_value(0, 0));
                s_smaller_then.push_back(middle_max_value(0, 0));
            }
            upper_boarder = findMinInVector(s_smaller_then);
            lower_boarder = findMaxInVector(s_bigger_then);
            if (upper_boarder < lower_boarder) {
                //    cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999, -9999);
            }
            return Point2d(lower_boarder, upper_boarder);

        } else {
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
            middle_max_value(0, 1) /= line.getDirectionVector()(0, 1);
            middle_min_value(0, 1) /= line.getDirectionVector()(0, 1);
            range_two.push_back(middle_max_value(0, 1));
            range_two.push_back(middle_min_value(0, 1));
            if (line.getDirectionVector()(0, 1) < 0) {
                s_bigger_then.push_back(middle_max_value(0, 1));
                s_smaller_then.push_back(middle_min_value(0, 1));
            } else {
                s_bigger_then.push_back(middle_min_value(0, 1));
                s_smaller_then.push_back(middle_max_value(0, 1));
            }
            middle_max_value(0, 2) /= line.getDirectionVector()(0, 2);
            middle_min_value(0, 2) /= line.getDirectionVector()(0, 2);
            range_three.push_back(middle_max_value(0, 2));
            range_three.push_back(middle_min_value(0, 2));
            if (line.getDirectionVector()(0, 2) < 0) {
                s_bigger_then.push_back(middle_max_value(0, 2));
                s_smaller_then.push_back(middle_min_value(0, 2));
            } else {
                s_bigger_then.push_back(middle_min_value(0, 2));
                s_smaller_then.push_back(middle_max_value(0, 2));
            }
            upper_boarder = findMinInVector(s_smaller_then);
            lower_boarder = findMaxInVector(s_bigger_then);
            if (upper_boarder < lower_boarder) {
//                cout << "The line is Out of the Grid" << endl;
                return Point2d(-9999, -9999);
            }
        }
        return Point2d(lower_boarder, upper_boarder);
    }

    /*!
     * find minimum in a given vector
     * @param vec - the vector.
     * @return minimum of the vector
     */
    float findMinInVector(vector<float> vec) {
        float min = vec[0];
        for (int i = 0; i < vec.size(); i++) {
            if (vec[i] < min) {
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
    float findMaxInVector(vector<float> vec) {
        float max = vec[0];
        for (int i = 0; i < vec.size(); i++) {
            if (vec[i] > max) {
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
    tuple<Matx31f, Matx31f> findIntersectionPoint(straight_line_equation line, Point2d S) {
        Matx31f entrancePoint = line.getPoint() + S.x * line.getDirectionVector();
        Matx31f exitPoint = line.getPoint() + S.y * line.getDirectionVector();
        tuple<Matx31f, Matx31f> tup(entrancePoint, exitPoint);
        return tup;
    }

    /*!
    * this function gets coordinates in the grid axis system, and returns the indexes of the voxel from the memory(the key of the map),
    * that satisfies that the coordinates fall on its area,
      if the coordinates fall in the area of two voxel (if one of the coordinates
    * is exactly step / 2 , we will return both of them), thats why we return a vector.
    * @param i - the x coordinates
    * @param j - the y coordinates
    * @param k - the z coordinates
    * @return - the voxels indexes as a vecotr of tuples.
    */
    vector<tuple<int,int,int>> getVoxelIndexesFromCoordinates(float i, float j, float k){
        /// in case the point is out of the grid
        vector<tuple<int, int, int>> keys;
        if (fabs(i) > (EDGE_COORDINATE + 0.5) * step_size || fabs(j) > (EDGE_COORDINATE + 0.5) * step_size ||
            fabs(k) > (EDGE_COORDINATE + 0.5) * step_size) {
//            cout << "coordinates is out of the grid's bound" << endl;
            return keys;
        }

        float x = i / step_size;
        float y = j / step_size;
        float z = k / step_size;

        int floor_x = 0, floor_y = 0, floor_z = 0, upper_x = 0, upper_y = 0, upper_z = 0;
        bool flag_x = false, flag_y = false, flag_z = false; /// this flags denothing wheter the coordinate it in between two voxels
        if (fabs(x - (int) x) == 0.5) {
            floor_x = (int) x;
            if (x >= 0) {
                upper_x = (int) x + 1;
            } else {
                upper_x = (int) x - 1;
            }
            flag_x = true;
        }
        if (fabs(y - (int) y) == 0.5) {
            floor_y = (int) y;
            if (y >= 0) {
                upper_y = (int) y + 1;
            } else {
                upper_y = (int) y - 1;
            }
            flag_y = true;
        }
        if (fabs(z - (int) z) == 0.5) {
            floor_z = (int) z;
            if (z >= 0) {
                upper_z = (int) z + 1;
            } else {
                upper_z = (int) z - 1;
            }
            flag_z = true;
        }
        if (abs(x - (int) x) < 0.5) {
            x = (int) x;
        } else {
            if (x >= 0) x = (int) x + 1;
            else x = (int) x - 1;
        }
        if (fabs(y - (int) y) < 0.5) {
            y = (int) y;
        } else {
            if (y >= 0) y = (int) y + 1;
            else y = (int) y - 1;
        }

        if (fabs(z - (int) z) < 0.5) {
            z = (int) z;
        } else {
            if (z >= 0) z = (int) z + 1;
            else z = (int) z - 1;
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
            } else if (fabs(floor_x) > EDGE_COORDINATE) {
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
            } else {
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
        } else {
            tuple<int, int, int> key(x, y, z);
            keys.push_back(key);
        }
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
    vector<tuple<int, int, int>> getVoxelFromCoordinatesOrPush(float i, float j, float k, straight_line_equation line, int frame_number) {
        vector<tuple<int, int, int>> keys;
        keys = getVoxelIndexesFromCoordinates(i, j, k);
        tuple<int,int,int> t(15,-6,-32);
        if(!keys.empty()) {
            if (keys.back() == t) {
                int stop = 1;
            }
        }
        UpdateVoxelsFromIndexes(keys, line, frame_number);
        return keys;

    }

    /*!
     * this function is responsible for conducting the bresenham algorithim in order to color the
     * relevant voxels, the voxels that the ray go trough them.
     * @param line - the equation of the ray
     * @param entrance_voxel - the first voxel that the ray touches when it enter the grid range.
     */
    void bresenhamAlgorithim(straight_line_equation line, tuple<int, int, int> entrance_voxel, int frame_number) {
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
                get<0>(next_voxel) += (direction_x >= 0) ? 1 : -1;
                /// in case we got to a voxel which is out of the grid we've finished.
                next_point_x = get<0>(next_voxel) * step_size;
                /// finding S
                slope = (next_point_x - line.getPoint()(0)) / direction_x;
                /// finding next y and next z
                next_point_y = line.getPoint()(1) + slope * direction_y;
                next_point_z = line.getPoint()(2) + slope * direction_z;
                /// in case this vector is empty it means that the next voxel is out of the grid and we should break
                auto vec = getVoxelFromCoordinatesOrPush(next_point_x, next_point_y, next_point_z, line, frame_number);
                if (vec.empty()) {
                    break;
                }

            }
                /// case the dominant direction is y
            else if (dominant_direction == fabs(direction_y)) {
                get<1>(next_voxel) += direction_y >= 0 ? 1 : -1;
                /// in case we got to a voxel which is out of the grid we've finished.
                next_point_y = get<1>(next_voxel) * step_size;
                /// finding S
                slope = (next_point_y - line.getPoint()(1)) / direction_y;
                /// finding next y and next z
                next_point_x = line.getPoint()(0) + slope * direction_x;
                next_point_z = line.getPoint()(2) + slope * direction_z;
                /// in case this vector is empty it means that the next voxel is out of the grid and we should break
                auto vec = getVoxelFromCoordinatesOrPush(next_point_x, next_point_y, next_point_z, line, frame_number);
                if (vec.empty()) {
                    break;
                }
            }
                /// case the dominant direction is z
            else if (dominant_direction == fabs(direction_z)) {
                get<2>(next_voxel) += direction_z >= 0 ? 1 : -1;;
                /// in case we got to a voxel which is out of the grid we've finished.
                next_point_z = get<2>(next_voxel) * step_size;
                /// finding S
                slope = (next_point_z - line.getPoint()(2)) / direction_z;
                /// finding next y and next z
                next_point_x = line.getPoint()(0) + slope * direction_x;
                next_point_y = line.getPoint()(1) + slope * direction_y;
                auto vec = getVoxelFromCoordinatesOrPush(next_point_x, next_point_y, next_point_z, line, frame_number);
                /// in case this vector is empty it means that the next voxel is out of the grid and we should break
                if (vec.empty()) {
                    break;
                }
            }
        }
    }
    /*!
     * function that checks the number of features in the grid (that was inserted as voxels)
     * this function is for testing purposes
     */
    void checkHowManyFeaturesInGrid(){
        int counter = 0;
        for(auto it = grid.begin(); it != grid.end(); it++){
            if(it->second.getIsFeature()){
                counter++;
            }
        }
        cout << "number of features is: " << counter << endl;

    }


    /*!
    * function that returns the coordinates in the grid axis's system for a given voxel
    * @param i - the i index of the voxel
    * @param j - the j index of the voxel
    * @param k - the k index of the voxel
    * @return coordiantes as Matx31f
    */
    Matx31f getCoordinatesFromVoxelIndex(int i, int j, int k) {
        Matx31f coordinates(i * step_size, j * step_size, k * step_size);
        return coordinates;
    }

    /*!
     * function that get vector of indexes and update the matching voxel distance and confidence,
     * if the voxel hasent been painted yet, we will create one and insert it to the grid.
     * @param keys - vector<tuple>, the indexes
     * @param line - the line that are coloring the the voxel we want to update.
     */
    void UpdateVoxelsFromIndexes(vector<tuple<int, int, int>> &keys, straight_line_equation line, int frame_number) {
        for (int i = 0; i < keys.size(); ++i) {
            tuple<int,int,int> tup(15, -6, -32);
            auto it = this->grid.find(keys[i]);
            if(tup == it->first){
                int stop = 1;
            }
            Matx31f point = getCoordinatesFromVoxelIndex(get<0>(keys[i]), get<1>(keys[i]), get<2>(keys[i]));
            Matx31f direction_vector = line.getDirectionVector();
            float v_x = direction_vector(0, 0);
            float v_y = direction_vector(1, 0);
            float v_z = direction_vector(2, 0);
            float s_voxel = 0;
            if (v_x != 0) {
                s_voxel = (point(0, 0) - line.getPoint()(0, 0)) / v_x;
            } else if (v_y != 0) {
                s_voxel = (point(1, 0) - line.getPoint()(1, 0)) / v_y;

            } else {
                s_voxel = (point(2, 0) - line.getPoint()(2, 0)) / v_z;

            }
            /// in case the voxel does not exist, we create it and insert it into the gird, then returning it.
            if (it == this->grid.end()) {
                voxel inserted_voxel;
                float confidence = calc_confidence(s_voxel, 1, inserted_voxel);
                float distance = calc_distance(s_voxel, 1, inserted_voxel);
//                cout << get<0>(keys[i]) << ", " << get<1>(keys[i]) << ", " << get<2>(keys[i]) << endl;
//                cout << "DISTANCE: " <<  distance << endl << endl;
                inserted_voxel.setConfidence(confidence);
                inserted_voxel.setDistance(distance);
                inserted_voxel.setFrameNumber(frame_number);
                this->grid.insert(pair<tuple<int, int, int>, voxel>(keys[i], inserted_voxel));
            }
                // in case the voxel have already been colored once, we just update the confidence and the distance
            else {
                float confidence = calc_confidence(s_voxel, 1, get<1>(*it));
                float distance = calc_distance(s_voxel, 1, get<1>(*it));
                get<1>(*it).setConfidence(confidence);
                get<1>(*it).setDistance(distance);
//                cout << get<0>(it->first) << ", " << get<1>(it->first) << ", " << get<2>(it->first) << endl;
//                cout << " UPDATED DISTANCE: " <<  it->second.getDistance() << endl << endl;
            }
        }
    }


    /*!
 * function for calculating the confidence of a given voxel,
 * the calculation is diffrent if it already has a confidence or it dosent.
 * @param s_voxel - the scalar that if we will multiply the direction vector by him, we will get to the voxel.
 * @param s_point - the scalar that if we will multiply the direction vector by him, we will get to the point that created the line(the feature)
 * @param vox - the voxel we're calculating the distance
 * @return - the confidence of the voxel.
 */
    float calc_confidence(float s_voxel, float s_point, voxel& vox) {
        float new_confidence = exp(-(pow(s_voxel - s_point, 2) / (SIGMA * SIGMA)));
        // means the voxel hasent been blacked yet
        if (vox.getConfidence() == -9999) {
            // means its the feature.
            if(s_voxel == 1){
                vox.setIsFeature();
            }
            return new_confidence;
        } else {
//            cout << " *****************************************************" << 5 + vox.getConfidence() << endl;
            if(new_confidence + vox.getConfidence() > MAX_CONFIDENCE){
                return MAX_CONFIDENCE;
            };
            return new_confidence + vox.getConfidence();
            //return (new_confidence + vox.getConfidence()) / 2;
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
    float calc_distance(float s_voxel, float s_point, voxel vox) {
        float new_confidence = exp(-(pow(s_voxel - s_point, 2) / (SIGMA * SIGMA)));
        // means the voxel hasent been blacked yet
        if (vox.getDistance() == 9999) {

            float distance = tanh(ALPHA * (s_voxel - s_point));
            if(distance == 0){
                distance += 0.0005;
            }
            return distance;
        } else {
            float d_new = tanh(ALPHA * (s_voxel - s_point));
            if((vox.getConfidence() + new_confidence) == 0){
                return d_new;
            }
            float new_distance = (vox.getDistance() * vox.getConfidence() + d_new * new_confidence) / (vox.getConfidence() + new_confidence);
            if(new_distance > MAX_DISTANCE){
                new_distance = MAX_DISTANCE;
            }
            if(new_distance < MIN_DISTANCE){
                new_distance = MIN_DISTANCE;
            }
//            new_distance = tanh(NORMALIZE_DISTANCE_FACTOR * (new_distance));
   //         float new_distance = vox.getDistance() + tanh(ALPHA * (s_voxel - s_point));
//            if(new_distance == 0){


//                    vox.getDistance() * vox.getConfidence() + tanh(ALPHA * (s_voxel - s_point)) * new_confidence;
//            cout << "DISTANCE " << new_distance << endl;

            return new_distance;
        }
    }

    /*!
     * helper function for creating a text file out of the grid, prepare the data in a vector of vectors
     * where each vector resembles to a line in the following format
     * i , j , k , distance, confidence.
     * @param lines - the output vector.
     */
    void createLines(vector<vector<float>> &lines) {
        vector<float> line;
        for (auto it = grid.begin(); it != grid.end(); it++) {
            /// push i, j, k
            line.push_back(get<0>(it->first));
            line.push_back(get<1>(it->first));
            line.push_back(get<2>(it->first));
            /// push confidence, distance
            line.push_back(it->second.getDistance());
            line.push_back(it->second.getConfidence());
            lines.push_back(line);
            line.clear();
        }
    }
    /*!
     * function for creating a text file containing information regarding each of the grid's voxels
     * the format will be: each voxel in each line such that i, j , k , distance, confidence
     * @param path - path to the file to be created
     * @param lines - vector of vectors where each vector resembles to a line in the file.
     */
    void createTextFileFromGrid(string path, vector<vector<float>> &lines) {
        ofstream file(path);
        for (int i = 0; i < lines.size(); i++) {
            file << lines[i].operator[](0) << " " << lines[i].operator[](1) << " " << lines[i].operator[](2) << " "
                 << lines[i].operator[](3) << " " << lines[i].operator[](4) << '\n';
        }
    }




    /*!
     * function that get voxel by INDEX and return TRUE if this voxel inside the grid
     * or FALSE if outside the grid
     * @param voxel - the INDEX of voxel to check
     * @return - TRUE if this voxel inside the grid
     * or FALSE if outside the grid
     */
    bool checkIfVoxelInsideGrid(tuple<int, int, int> voxel) {
        int x = get<0>(voxel);
        int y = get<1>(voxel);
        int z = get<2>(voxel);

        return (abs(x) <= EDGE_COORDINATE && abs(y) <= EDGE_COORDINATE && abs(z) <= EDGE_COORDINATE);
    }

    /*!
     * function tjat get voxel by INDEX and fill vector with all the legal neigboors of this voxel.
     * (legal neighboor = neighboor who inside the grid)
     * @param voxel_indexes - the voxel that the function find his neigboors
     * @param neighboors - the vevtor that the function will fill
     */
    void findVoxelNeighboors(tuple<int, int, int> voxel_indexes, vector<tuple<int, int, int>> &neighboors) {

        int x = get<0>(voxel_indexes);
        int y = get<1>(voxel_indexes);
        int z = get<2>(voxel_indexes);

        /// all the neighboors in the same voxel's level
        tuple<int, int, int> t1(x, y + 1, z);
        if (checkIfVoxelInsideGrid(t1)) neighboors.push_back(t1);
        tuple<int, int, int> t2(x, y - 1, z);
        if (checkIfVoxelInsideGrid(t2)) neighboors.push_back(t2);
        tuple<int, int, int> t3(x + 1, y, z);
        if (checkIfVoxelInsideGrid(t3)) neighboors.push_back(t3);
        tuple<int, int, int> t4(x - 1, y, z);
        if (checkIfVoxelInsideGrid(t4)) neighboors.push_back(t4);
        tuple<int, int, int> t5(x + 1, y + 1, z);
        if (checkIfVoxelInsideGrid(t5)) neighboors.push_back(t5);
        tuple<int, int, int> t6(x - 1, y - 1, z);
        if (checkIfVoxelInsideGrid(t6)) neighboors.push_back(t6);
        tuple<int, int, int> t7(x + 1, y - 1, z);
        if (checkIfVoxelInsideGrid(t7)) neighboors.push_back(t7);
        tuple<int, int, int> t8(x - 1, y + 1, z);
        if (checkIfVoxelInsideGrid(t8)) neighboors.push_back(t8);

        /// all the neighboors in the up level from voxel's level
        tuple<int, int, int> t9(x, y + 1, z + 1);
        if (checkIfVoxelInsideGrid(t9)) neighboors.push_back(t9);
        tuple<int, int, int> t10(x, y - 1, z + 1);
        if (checkIfVoxelInsideGrid(t10)) neighboors.push_back(t10);
        tuple<int, int, int> t11(x + 1, y, z + 1);
        if (checkIfVoxelInsideGrid(t11)) neighboors.push_back(t11);
        tuple<int, int, int> t12(x - 1, y, z + 1);
        if (checkIfVoxelInsideGrid(t12)) neighboors.push_back(t12);
        tuple<int, int, int> t13(x + 1, y + 1, z + 1);
        if (checkIfVoxelInsideGrid(t13)) neighboors.push_back(t13);
        tuple<int, int, int> t14(x - 1, y - 1, z + 1);
        if (checkIfVoxelInsideGrid(t14)) neighboors.push_back(t14);
        tuple<int, int, int> t15(x + 1, y - 1, z + 1);
        if (checkIfVoxelInsideGrid(t15)) neighboors.push_back(t15);
        tuple<int, int, int> t16(x - 1, y + 1, z + 1);
        if (checkIfVoxelInsideGrid(t16)) neighboors.push_back(t16);
        tuple<int, int, int> t17(x, y, z + 1);
        if (checkIfVoxelInsideGrid(t17)) neighboors.push_back(t17);

        /// all the neighboors in the bottom level from voxel's level
        tuple<int, int, int> t18(x, y + 1, z - 1);
        if (checkIfVoxelInsideGrid(t18)) neighboors.push_back(t18);
        tuple<int, int, int> t19(x, y - 1, z - 1);
        if (checkIfVoxelInsideGrid(t19)) neighboors.push_back(t19);
        tuple<int, int, int> t20(x + 1, y, z - 1);
        if (checkIfVoxelInsideGrid(t20)) neighboors.push_back(t20);
        tuple<int, int, int> t21(x - 1, y, z - 1);
        if (checkIfVoxelInsideGrid(t21)) neighboors.push_back(t21);
        tuple<int, int, int> t22(x + 1, y + 1, z - 1);
        if (checkIfVoxelInsideGrid(t22)) neighboors.push_back(t22);
        tuple<int, int, int> t23(x - 1, y - 1, z - 1);
        if (checkIfVoxelInsideGrid(t23)) neighboors.push_back(t23);
        tuple<int, int, int> t24(x + 1, y - 1, z - 1);
        if (checkIfVoxelInsideGrid(t24)) neighboors.push_back(t24);
        tuple<int, int, int> t25(x - 1, y + 1, z - 1);
        if (checkIfVoxelInsideGrid(t25)) neighboors.push_back(t25);
        tuple<int, int, int> t26(x, y, z - 1);
        if (checkIfVoxelInsideGrid(t26)) neighboors.push_back(t26);
    }



    /*!
     * function that recives indexes of a voxel and create a vector of tuples that create all of its adjacent neighbors
     * in a GRIDCELL, so eventually it will hold 8 indexes including the given coordiantes.
     * we refer to the given coordiantes as if it was the bottom left voxel in the GRIDCELL and we add the other
     * voxels accordingly.
     * @param neighboor
     * @param coordinate
     */
    void checkBottomLeft(vector<tuple<int, int, int>> &neighboor, tuple<int, int, int> bottom_left_coordinate) {
        int x = get<0>(bottom_left_coordinate);
        int y = get<1>(bottom_left_coordinate);
        int z = get<2>(bottom_left_coordinate);
        /// each voxel can be part of 8 differnt grid cells

        /// top left
        tuple<int, int, int> right(x + 1, y, z);
        tuple<int, int, int> behind_right(x + 1, y + 1, z);
        tuple<int, int, int> behind(x, y + 1, z);
        tuple<int, int, int> up(x, y , z + 1);
        tuple<int, int, int> right_up(x + 1, y, z + 1);
        tuple<int, int, int> behind_right_up(x + 1, y + 1, z + 1);
        tuple<int, int, int> behind_up(x, y + 1, z + 1);
        if (abs(x + 1) <= EDGE_COORDINATE && abs(y - 1) <= EDGE_COORDINATE && abs(z - 1) <= EDGE_COORDINATE) {
            neighboor.push_back(bottom_left_coordinate);
            neighboor.push_back(right);
            neighboor.push_back(behind_right);
            neighboor.push_back(behind);
            neighboor.push_back(up);
            neighboor.push_back(right_up);
            neighboor.push_back(behind_right_up);
            neighboor.push_back(behind_up);



        }
    }

    /*!
     * function for finding all the indexes of vertices that create one GRIDCELL, we recive the indices of the
     * bottom left voxel, we retrive all the indcies of all the other voxels.
     * @param voxel_indexes - the indexes of the bottom left voxel in the GRIDCELL
     * @param neighboors - vector of tuples with all of the indices of the other voxels in the GRIDCELL
     */
    void findAllVertexes(tuple<int, int, int> voxel_indexes, vector<tuple<int, int, int>> &neighboors) {
        checkBottomLeft(neighboors, voxel_indexes);
    }

    /*!
     * function that through over the grid (map with black voxels) and for each voxel in grid
     * she will push the voxel with his neighboor, while if some neighboor without data we will
     * initialize him with same distance value of the orig voxel
     */
    void addVoxelsToDensifiedGrid() {
        vector<tuple<int, int, int>> neighboors;
        for (auto it = this->grid.begin(); it != this->grid.end(); ++it) {
            auto densifid_grid_it = densified_grid.find(it->first);
            // if the voxel hasent been inserted yet
            if (densifid_grid_it == densified_grid.end()) {
                densified_grid.insert(pair<tuple<int, int, int>, voxel>(it->first, it->second));
            }
            // find the neighboors
            findVoxelNeighboors(it->first, neighboors);
            for (int i = 0; i < neighboors.size(); ++i) {
                auto densifid_grid_it = densified_grid.find(neighboors[i]);
                // if the neighboor hasent been inserted yet we put it with the voxel distance and confidence
                if (densifid_grid_it == densified_grid.end()) {
                    float current_voxel_distance = it->second.getDistance();
                    float current_voxel_confidende = it->second.getConfidence();
                    auto grid_it = grid.find(neighboors[i]);
                    // if the neighboor is not part of the grid
                    if (grid_it == grid.end()) {
                        voxel voxel_to_insert(current_voxel_distance, current_voxel_confidende);
                        densified_grid.insert(pair<tuple<int, int, int>, voxel>(neighboors[i], voxel_to_insert));
                    } else {
                        voxel voxel_to_insert(grid_it->second.getDistance(), grid_it->second.getConfidence());
                        densified_grid.insert(pair<tuple<int, int, int>, voxel>(neighboors[i], voxel_to_insert));
                    }
                }
            }
            neighboors.clear();
        }
    }

    bool checkIfAnyNeighboorIsFeature(tuple<int,int,int> indexes_of_voxel){
        vector<tuple<int, int, int>> neighboors;
        findVoxelNeighboors(indexes_of_voxel, neighboors);
        for (int i = 0; i < neighboors.size(); ++i) {
            auto grid_iterator = grid.find(neighboors[i]);
            // if the neighboor hasent been inserted yet we put it with the voxel distance and confidence
            if (grid_iterator == densified_grid.end()) {
                continue;
            }
            else{
                if(grid_iterator->second.getIsFeature()){
                    return true;
                }
            }
        }
        return false;

    }
   /*!
    * function for converting our grid into GRIDCELLS, every 8 adjacent voxels become one gridcell
    * where every voxel will be a vertex of the gridcell (will be inserted into the vertex vector in the gridcell)
    * , and each voxel's distance will be inserted into the val vector inside the created gridcell
    */
    void createGridCellMap() {
        //int counter = densified_grid.size();
        for (auto it = densified_grid.begin(); it != densified_grid.end(); it++) {
            vector<tuple<int, int, int>> possible_grid_cells;
            findAllVertexes(it->first, possible_grid_cells);
            /// means at least one of the vertices is out of the grid, so we continue to the next one.
            if(possible_grid_cells.empty()){
//                counter--;
//                cout << counter << endl;
                continue;
            }
            /// the grid cell hasent been inserted yet
            else if (grid_cell_map.find(it->first) == grid_cell_map.end())
            {
                bool all_voxels_black = true;
                for(int i = 0; i < possible_grid_cells.size(); i++){
                    if(densified_grid.find(possible_grid_cells[i]) == densified_grid.end()){
                        all_voxels_black = false;
                        break;
                    }
                }
                /// in case all of the voxels are within the grid, but not all of them have been blacked we dont want
                /// to create a grid cell from them, we need all of them to be blacked.
                if(!all_voxels_black){
                    continue;
                }
                GRIDCELL new_cube(possible_grid_cells, this->getDensifiedGrid(), this);
                //new_cube.print();
                pair<tuple<int, int, int>, GRIDCELL> new_grid_cell(it->first, new_cube);
                grid_cell_map.insert(new_grid_cell);

            }
//            counter--;
//            cout << counter << endl;
        }
        cout <<" number of grid cells " <<  grid_cell_map.size() << endl;


    }
    /*!
     * retun grid step size
     * @return grid's step size
     */
    float getStepSize(){
        return step_size;
    }
    /*!
     * we remove voxels from the grid that their confidence is less then the determined threshold
     */
    void updateGridByConfidenceThreshold(){
        auto it = grid.begin();
        while(it != grid.end()){
            if(it->second.getConfidence() < THRESHOLD_CONFIDENCE /*&& !checkIfAnyNeighboorIsFeature(it->first)*/){
               it =  grid.erase(it);
            }
            else {
                it++;
            }
        }
    }

    /*!
     * function for printing all the voxel's confidence.
     */
    void printGridByConfidence(){
        for(auto it = grid.begin(); it != grid.end(); it++){
            cout << "indexes " << get<0>(it->first) << ", " << get<1>(it->first) << ", " << get<2>(it->first)
                    << " confidence " << it->second.getConfidence() <<  endl;
        }
    }


    /*!
 * function that find maximum confidence value between the black voxels and normalize the voxels's confidende value by
 * division by the maximum value. (turn all confidence values to [0,1] scale)
 * @param grid - the data structure when the black voxels exist
 */
    void normalizeVoxelsConfidence(){
        float max_confidence=-9999;
        for (auto it = grid.begin(); it != grid.end(); ++it) {
            if(it->second.getConfidence() > max_confidence) {
                max_confidence = it->second.getConfidence();
            }
        }
        for (auto it = grid.begin(); it != grid.end(); ++it) {
            float normalize_confidence = it->second.getConfidence() / max_confidence;
            it->second.setConfidence(normalize_confidence);
        }
    }
    void addFeaturesToGrid(vector<vector<worldPoint>>& frames_features) {
        for (int i = 0; i < frames_features.size(); i++) {
            for (int j = 0; j < frames_features[i].size(); j++) {
                Matx31f point_in_world(frames_features[i][j].x, frames_features[i][j].y, frames_features[i][j].z);
                Matx31f point_in_grid = mapFromWorldToGrid(point_in_world);
                if (fabs(point_in_grid(0, 0)) > (EDGE_COORDINATE + 0.5) * step_size ||
                    fabs(point_in_grid(0, 1)) > (EDGE_COORDINATE + 0.5) * step_size ||
                    fabs(point_in_grid(0, 2)) > (EDGE_COORDINATE + 0.5) * step_size) {
//            cout << "coordinates is out of the grid's bound" << endl;
                    continue;
                }
                vector<tuple<int, int, int>> features;
                if(i == 0 && j ==5){
                    int stop = 1;
                }
                features = getVoxelIndexesFromCoordinates(point_in_grid(0, 0), point_in_grid(0, 1),
                                                          point_in_grid(0, 2));

                for (int t = 0; t < features.size(); t++) {
                    auto it = grid.find(features[t]);
                    if (it == grid.end()) {
                        voxel inserted_voxel;
                        float confidence = calc_confidence(1, 1, inserted_voxel);
                        float distance = calc_distance(1, 1, inserted_voxel);
                        inserted_voxel.setConfidence(confidence);
                        inserted_voxel.setDistance(distance);
                        this->grid.insert(pair<tuple<int, int, int>, voxel>(features[t], inserted_voxel));
                    }
                        // in case the voxel have already been colored once, we just update the confidence and the distance
                    else {
                        float confidence = calc_confidence(1, 1, get<1>(*it));
                        float distance = calc_distance(1, 1, get<1>(*it));
                        it->second.setConfidence(confidence);
                        it->second.setDistance(distance);
//                cout << get<0>(it->first) << ", " << get<1>(it->first) << ", " << get<2>(it->first) << endl;
//                cout << " UPDATED DISTANCE: " <<  it->second.getDistance() << endl << endl;
                    }
                }
            }
        }
    }

    /*!
 * function that get voxels indexs, convert them to coordinates in the word and put them in vector who represent voxel details
 * in grid.txt file
 * @param indexs - voxel indexs in gris axis
 * @param line - vector who represent detaile on some voxel
 * @param grid - the data structure when the black voxels exist
 */
    vector<float> convertVoxelIndexToCoordinateInTheWorld(tuple<int,int,int> indexs){
        vector<float> XYZpoints;
        Matx31f grid_coordinates_in_world = mapFromGridToWorld(get<0>(indexs), get<1>(indexs), get<2>(indexs));
        XYZpoints.push_back(grid_coordinates_in_world(0,0));
        XYZpoints.push_back(grid_coordinates_in_world(1,0));
        XYZpoints.push_back(grid_coordinates_in_world(2,0));
        return XYZpoints;
    }



    /*!
 * function for projecting the grid voxels into all the frames (the voxels of the grid are only the one that have been blacked by a ray)
 * @param grid - the grid we project
 * @param grid_edges - a vector contating all the coordinates of the grid's edges
 * @param frames - the frames we projct the grid to.
 * @param images_path - paths to all the images in the local machine.
 */
    void projectGridToFrames(vector<Point3d>& grid_edges, vector<Frame>& frames, vector<string>& images_path){
        int cntr=0;
        for(int i = 0; i < frames.size(); i++){
            projectpointsFromGridToImage(images_path[i], grid_edges, grid, &frames[i], "Frame" + to_string(i), &cntr);
        }
    }

/*!
 * function for creating a visualiztion of the confidence of all the voxels, the bigger the confidence of the voxel,
 * it will be presented by a bigger circle in the image, we do it for 50 diffrent frames.
 * @param grid - the grid we project
 * @param grid_edges - a vector contating all the coordinates of the grid's edges
 * @param frames - the frames we projct the grid to.
 * @param images_path - paths to all the images in the local machine.
 */
    void createConfidenceHeatMap( vector<Point3d>& grid_edges, vector<Frame>& frames, vector<string>& images_path){
        int cntr=0;
        for(int i = 0; i < frames.size(); i++){
            createConfidenceHeatMap(images_path[i], grid_edges, grid, &frames[i], "Frame " + to_string(i), &cntr);
        }

    }


};

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
 * function that get all the Frame's features and return the average of them, that will be the grid center
 * that is created from the frame.
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


GRIDCELL::GRIDCELL(vector<tuple<int,int,int>>& indexes_of_vertexes, map<tuple<int,int,int>, voxel>& map, grid3D* grid){
    for(int i = 0; i < indexes_of_vertexes.size(); i++){
        p[i] = grid->mapFromGridToWorld(get<0>(indexes_of_vertexes[i]), get<1>(indexes_of_vertexes[i]), get<2>(indexes_of_vertexes[i]));
        val[i] = map.find(indexes_of_vertexes[i])->second.getDistance();
    }
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

#endif //NC_RD_FACEMASK_GRID3D_H
