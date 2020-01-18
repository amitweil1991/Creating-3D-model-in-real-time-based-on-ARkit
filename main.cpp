

#include "grid3D.h"
#include "marchingCubes.h"
#include "exportToFiles.h"
#define FEATURES_NUM_TO_PRESENT 1
#define STARTING_FEATURE_TO_PRESENT 0
using json = nlohmann::json;


void testProjection(vector<worldPoint> image1_first_features, vector<string> imagesPath,vector<Frame> frames, int* cntr);
Matx31f calc_avg_grid_center(vector<worldPoint> features_point);


/** function for presenting all the rays from all the  frames (each vector in features is a vector of features from a different frame)
 *
 * @param cam_poses - the cam poses in the GRID axis, together with the features it creates the rays
 * @param frames_featurs - the features that creates the rays
 * @param grid - the given grid we're presenting the rays on
 */
void CreateRaysAndVoxels(
        vector<Matx31f>& cam_poses,
        vector<vector<worldPoint>>& frames_featurs,
        grid3D& grid,
        int number_of_frames_to_present)
{

        vector<vector<tuple<int, int, int>>> keys;
        for (int i = 0; i < frames_featurs.size(); i++) {
            for (int j = 0; j <  frames_featurs[i].size(); ++j) {
                Matx31f point_in_world = convertWorldPointToMatx(frames_featurs[i][j]);
                Matx31f point_in_grid = grid.mapFromWorldToGrid(point_in_world);
                straight_line_equation line(cam_poses[i-1], point_in_grid);
                Point2d slope = grid.getSlopeRange(line);
                if (slope.x == -9999 && slope.y == -9999) {
                    continue;
                }
                tuple<Matx31f, Matx31f> intersection_points = grid.findIntersectionPoint(line, slope);
                Matx31f entrance_point = get<0>(intersection_points);
            /// insert feature
                keys.push_back(grid.getVoxelFromCoordinatesOrPush(entrance_point(0, 0), entrance_point(1, 0),entrance_point(2, 0),line ,i));
            // if no entrance point
                if (keys.back().empty()) {
                    continue;
                }
                grid.UpdateVoxelsFromIndexes(keys.back(), line, i);
            // push the feature itself.
                grid.bresenhamAlgorithim(line, keys.back()[0], i);
            }
        }
}



int main(int argc, char **argv){
    vector<string> jsons;
    vector<string> images;
    vector<vector<worldPoint>> frames_features;
    vector<Frame> frames_vector;
    /// parse arKit data
    ParseArKitData(images, jsons, frames_features, frames_vector, "jeep_scan");
    vector<string> images_paths;
    /// create grid
    grid3D grid = createGridFromGivenFrame(frames_features[0], frames_vector[0], 0.002);
    vector<Matx<float, 3, 1>> grid_edges;
    /// create ply from the features
    createPlyFileOfAllFeatures(frames_features, "grid_ply/features.ply");
    /// create ply for grid edges
    createPlyFromGridEdges(grid, grid_edges, "grid_ply/grid_boundries.ply");
    /// creating cam poses for all the frames in the grid axis
    vector<Matx31f> cam_poses;
    /// create ply for cam poses
    createCamPoseVectorAndPlyForFrames(cam_poses,frames_vector, grid, "grid_ply/cam_poses.ply");
    /// create rays from all features in all frames
    int number_of_frames_to_present = 50;
    grid.addFeaturesToGrid(frames_features);
    CreateRaysAndVoxels(cam_poses, frames_features, grid, number_of_frames_to_present);
    cout << "the number of voxels in the grid is: " << grid.getGrid().size() << endl;
    /// convert confidence values to [0,1] scale by normalizing them
    grid.normalizeVoxelsConfidence();
    cout << "start update grid by confidence (remove voxels with confidence lower than the threshold)" << endl;
    grid.updateGridByConfidenceThreshold();
    cout << "end update grid by confidence" << endl;
    cout << "the number of voxels in the updated is " << grid.getGrid().size() << endl;
    /// create grid.ply file for Mesh Lab
    exportGridToFile(grid, "grid_ply/grid.ply");
    exportGridDistancesToFile(grid, "grid_ply/grid_distances.ply");
        /// export grid to a text file
    vector<vector<float>> lines;
    grid.createLines(lines);
    grid.createTextFileFromGrid("grid_text/grid_as_text.txt", lines);
    cout << "start densify" << endl;
    /// densify the grid's data
    grid.addVoxelsToDensifiedGrid();
    cout << "finished densify" << endl;
    /// create grid cell map
    cout << " start create grid cell map" << endl;
    grid.createGridCellMap();
    cout << " finished create grid cell map" << endl;
    /// create mesh from grid
    vector<Mesh> grid_mesh;
    cout << " start create mesh from grid" << endl;
    CreateMeshFromGrid(grid_mesh, grid);
    cout << " finished create mesh from grid" << endl;
    /// create full mesh
    Mesh full_mesh;
    cout << " start create full mesh" << endl;
    createFullMesh(grid_mesh, full_mesh);
    cout << " finished create full mesh" << endl;
    /// export mesh to file
    string path_to_file("grid_ply/grid_as_mesh.off");
    cout << " start exporting mesh to file " << endl;
    exportMeshToFile(full_mesh, path_to_file);
    cout << " finished exporting mesh to file " << endl;
    cout << "create configuration file" << endl;
    string path_to_configuration_file("configuration_file/configuration_file.txt");
    /// create configuration file for this execution
    createConfigurationFile(path_to_configuration_file, grid, number_of_frames_to_present, FEATURES_NUM_TO_PRESENT,
            true, grid_mesh.size());


    return 0;
}


