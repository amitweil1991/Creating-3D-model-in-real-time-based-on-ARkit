cmake_minimum_required(VERSION 3.1)
project(untitled)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES main.cpp Frame.h  grid3D.h)
add_executable(opencv ${SOURCE_FILES})

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv ${OpenCV_LIBS})



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
 * function for creating a vector that contains all the grid's edges
 * @param vec - the vector contatining all the grid edges
 */
void createGridEdges(grid3D& grid, vector<Matx<float, 3, 1>>& vec){
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
    std::ofstream outFile("grid_ply/grid_boundries.ply");
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


