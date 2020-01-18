//
// Created by amitw on 15/01/2020.
//

#ifndef UNTITLED_UTILITIES_H
#define UNTITLED_UTILITIES_H

#include "Frame.h"


using namespace std;
using namespace cv;







/*!
 * function for parsing the Arkit data (json files), and create frames instances out of it, vetctor with all
 * the images names, json file names, vector with all the created frames, and vector of vectors containing all the
 * features arkit provided us.
 *
 * *** IN ORDER TO USE THE FUNCTION, THE ARKIT DATA SHOULD BE IN THE PROJECT LOCATION ****
 *
 * @param images_files_names - a vector that will hold all the images names.
 * @param json_files_names - a vector that will hold all the json files names
 * @param frames_features - vector of vectors, where each vector holds all of the features of a specific frame.
 * @param frames_vector - vector containing all the created frames instances.
 */
void ParseArKitData(vector<string>& images_files_names, vector<string>& json_files_names,
                    vector<vector<worldPoint>>& frames_features, vector<Frame>& frames_vector, string path_to_dir)
{
    string command1 = "cd " + path_to_dir;
    string command2 = "dir *.jpg > images.txt";
    string command3 = "dir *.json > jsons.txt";
    int n1 = command1.length();
    int n2 = command2.length();
    int n3 = command2.length();
    char command_char_array1[n1 + 1];
    char command_char_array2[n2 + 1];
    char command_char_array3[n2 + 1];
    // copying the contents of the
    // string to char array
    strcpy(command_char_array1, command1.c_str());
    system(command_char_array1);
    strcpy(command_char_array2, command2.c_str());
    system(command_char_array2);
    strcpy(command_char_array3, command3.c_str());
    system(command_char_array3);
    ifstream images("images.txt");
    ifstream jsons("jsons.txt");
    string line_image;
    // parse images
    while(getline(images, line_image)){
        istringstream parse_line(line_image);
        string image_file;
        while(parse_line >> image_file){
            images_files_names.push_back(image_file);
        }
    }
    // parse jsons
    string line_jsons;
    while(getline(jsons, line_jsons)){
        istringstream parse_line(line_jsons);
        string json_file;
        while(parse_line >> json_file){
            json_files_names.push_back(json_file);
        }
    }
    // parse the jsons, create frames and feature vectors.
    for(int i = 0; i < json_files_names.size(); i++){
        vector<float> image_C1;
        vector<float> image_C2;
        vector<worldPoint> image_first_features;
        parseJson(json_files_names[i], &image_C1, &image_C2, &image_first_features);
        Frame frame(image_C1,image_C2);
        frames_vector.push_back(frame);
        frames_features.push_back(image_first_features);
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
 * function that get voxel's distance and return his RGB value. the RGB value is more red if the confidence bigger
 * @param confidence
 * @param RGBvalues
 */
vector<int> getRGBbyDistance(float distance){
    vector<int> RGBvalues;
    RGBvalues.push_back(255 - (distance));
    RGBvalues.push_back(255 - (2 * 1000 * abs(distance - 0.5)));
    RGBvalues.push_back(255 * distance * 1000);
    return RGBvalues;
}


/*!
 * function to create the video from all the images
 */
void video (string path_to_video, string path_to_dir, int frame_num)
{
    VideoWriter out_capture(path_to_video, VideoWriter::fourcc('M', 'J', 'P', 'G'), 3,
                            Size(1920, 1440));
    for (int i = 0; i < frame_num; i++) {

        VideoCapture in_capture(path_to_dir + to_string(i)+".jpg");
        Mat img;

        while (true) {
            in_capture >> img;
            if (img.empty())
                break;

            out_capture.write(img);
        }
    }
}




#endif //UNTITLED_UTILITIES_H
