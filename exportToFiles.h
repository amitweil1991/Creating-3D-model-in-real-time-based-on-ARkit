//
// Created by Amit weil on 15/01/2020.
//

#ifndef UNTITLED_EXPORTTOFILES_H
#define UNTITLED_EXPORTTOFILES_H

#include "grid3D.h"


/******************************************************************************
 * this header contains all the function that export information out to files *
*******************************************************************************/




/*!
* function that generate grid.ply with all the details of black voxels
* @param grid - the data structure when the black voxels exist
*/
void exportGridToFile(grid3D& grid, string path){
//        cout << grid.size() << endl;
    std::ofstream outFile(path);
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
            XYZpoints = grid.convertVoxelIndexToCoordinateInTheWorld(it->first);
            RGBvalues = getRGBbyConfidance(it->second.getConfidence());
            outFile << to_string(XYZpoints[0]) << " " << to_string(XYZpoints[1]) << " " << to_string(XYZpoints[2])
                    << " 0 0 0 "
                    << to_string(RGBvalues[0]) << " " << to_string(RGBvalues[1]) << " " << to_string(RGBvalues[2])
                    << endl;
            XYZpoints.clear();
            RGBvalues.clear();
        }
        else {
            int check = 1;
        }
    }
    outFile.close();
}



/** function for creating a vector with all the camera positions in a given grid axis, from the given frames **/
void createCamPoseVectorAndPlyForFrames(vector<Matx31f>& cam_pose_vector, vector<Frame>& frames, grid3D& grid, string path ) {
    vector<Matx31f> campose_world;
    for (int i = 0; i < frames.size(); i++) {
        cam_pose_vector.push_back(grid.mapFromWorldToGrid(frames[i].getCampos()));
        campose_world.push_back(frames[i].getCampos());
//        cout << "camPose of frame " << i << " is: ";
//        cout << "grid: " << grid.mapFromWorldToGrid(frames[i].getCampos()) << endl;
//        cout << "World: " << frames[i].getCampos() << endl;
    }

    std::ofstream outFile(path);
    outFile << "ply" << endl;
    outFile << "format ascii 1.0" << endl;
    outFile << "element vertex " + to_string(campose_world.size()) << endl;
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
    for (int i = 0; i < cam_pose_vector.size(); i++) {
        outFile << to_string(campose_world[i](0, 0)) << " " << to_string(campose_world[i](0, 1)) << " "
                << to_string(campose_world[i](0, 2))
                << " 0 0 0 "
                << to_string(255) << " " << to_string(255) << " " << to_string(255)
                << endl;
    }

    outFile.close();
}

/*!
 * function for creating ply file from the grid center so we can see it on mesh lab.
 * @param grid
 * @param path_to_ply
 */
void createPlyFromGridCenter(grid3D &grid, string path_to_ply){
    std::ofstream outFile(path_to_ply);
    outFile << "ply" << endl;
    outFile << "format ascii 1.0" << endl;
    outFile << "element vertex " + to_string(1) << endl;
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
    outFile << to_string(grid.getPw()(0, 0)) << " " << to_string(grid.getPw()(0, 1)) << " " << to_string(grid.getPw()(0, 2))
            << " 0 0 0 "
            << to_string(255) << " " << to_string(255) << " " << to_string(255)
            << endl;
    outFile.close();
}



/*!
    * function that creates a ply file from all of the features we get from Arkit
    * @param frames_features - the features
    * @param path_to_ply - path to the generated ply file.
    */
void createPlyFileOfAllFeatures(vector<vector<worldPoint>>& frames_features, string path_to_ply){
    int features_num = 0;
    for(int i = 0; i <  frames_features.size(); i++){
        for(int j = 0; j < frames_features[i].size(); j++){
            features_num++;
        }
    }
//        cout << "feature number from data is " << features_num << endl;
    std::ofstream outFile(path_to_ply);
    outFile << "ply" << endl;
    outFile << "format ascii 1.0" << endl;
    outFile << "element vertex " + to_string(features_num) << endl;
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
    for(int i = 0; i <  frames_features.size(); i++){
        for(int j = 0; j < frames_features[i].size(); j++){
            features_num++;
            outFile << to_string(frames_features[i][j].x) << " " << to_string(frames_features[i][j].y) << " " << to_string(frames_features[i][j].z)
                    << " 0 0 0 "
                    << to_string(255) << " " << to_string(255) << " " << to_string(255)
                    << endl;
        }
    }
    outFile.close();

}

/*!
 * function creating configuration file with information regarding the execution and its components
 * the information is:
 * 1. size of the grid
 * 2. number of voxels in the grid.
 * 3. number of voxels in the densified version of the grid.
 * 4. step size that was configured in the grid.
 * 5. the confidence threshold that was configured in this execution
 * 6. SIGMA's value
 * 7. ALPHA's value
 * 8. the number of frames that we used in this execution
 * 9. the number of features we presented from each frame (same number for all of them)
 * 10. number of meshes that was created in this execution
 *
 * @param path - path to the file created
 * @param grid - the grid that was used in the execution
 * @param frame_num - number of frames that was presented
 * @param number_of_features_from_frame - number of features we presented from each frame
 * @param all - boolean that denoting if we presented all of the features from each frame (true or false accordingly)
 * @param number_of_meshes_created - the number of meshes that was created eventualy.
 */
void createConfigurationFile(string path, grid3D& grid, int frames_num, int number_of_features_from_frame, bool all,
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
    myfile << "number of frames presented:  " << frames_num << '\n';
    if(all){
        myfile << "number of features from each frame:  " << "all" << '\n';
    }
    else{
        myfile << "number of features from each frame:  " << number_of_features_from_frame << '\n';
    }
    myfile << "number of meshes created:  " << number_of_meshes_created << '\n';
    myfile.close();


}



/*!
 * function for creating a vector that contains all the grid's edges
 * @param vec - the vector contatining all the grid edges
 */
void createPlyFromGridEdges(grid3D& grid, vector<Matx<float, 3, 1>>& vec, string path ){
    Point3d p1(-EDGE_COORDINATE,-EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p3(-EDGE_COORDINATE,EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p2(EDGE_COORDINATE,-EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p4(EDGE_COORDINATE,EDGE_COORDINATE,-EDGE_COORDINATE);
    Point3d p5(-EDGE_COORDINATE,-EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p6(EDGE_COORDINATE,-EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p7(-EDGE_COORDINATE,EDGE_COORDINATE,EDGE_COORDINATE);
    Point3d p8(EDGE_COORDINATE,EDGE_COORDINATE,EDGE_COORDINATE);
    vec.push_back(grid.mapFromGridToWorld(-EDGE_COORDINATE,-EDGE_COORDINATE,-EDGE_COORDINATE));
    vec.push_back(grid.mapFromGridToWorld(-EDGE_COORDINATE,EDGE_COORDINATE,-EDGE_COORDINATE));
    vec.push_back(grid.mapFromGridToWorld(EDGE_COORDINATE,-EDGE_COORDINATE,-EDGE_COORDINATE));
    vec.push_back(grid.mapFromGridToWorld(EDGE_COORDINATE,EDGE_COORDINATE,-EDGE_COORDINATE));
    vec.push_back(grid.mapFromGridToWorld(-EDGE_COORDINATE,-EDGE_COORDINATE,EDGE_COORDINATE));
    vec.push_back(grid.mapFromGridToWorld(EDGE_COORDINATE,-EDGE_COORDINATE,EDGE_COORDINATE));
    vec.push_back(grid.mapFromGridToWorld(-EDGE_COORDINATE,EDGE_COORDINATE,EDGE_COORDINATE));
    vec.push_back(grid.mapFromGridToWorld(EDGE_COORDINATE,EDGE_COORDINATE,EDGE_COORDINATE));
    std::ofstream outFile(path);
    outFile << "ply" << endl;
    outFile << "format ascii 1.0" << endl;
    outFile << "element vertex " + to_string(vec.size()) << endl;
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
    for(int i = 0; i < vec.size(); i++){
        outFile << to_string(vec[i](0,0)) << " " << to_string(vec[i](0,1)) << " " << to_string(vec[i](0,2))
                << " 0 0 0 "
                << to_string(255) << " " << to_string(255) << " " << to_string(255)
                << endl;

    }
    outFile.close();
}





#endif //UNTITLED_EXPORTTOFILES_H
