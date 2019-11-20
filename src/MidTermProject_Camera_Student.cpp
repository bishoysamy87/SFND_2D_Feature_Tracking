/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)
	std::ofstream myfile;
	myfile.open ("./output/final_result.csv");
	/** File header*/
	myfile << "img_scenario,detector,descriptor,keypoint_numbers,time_taken_detector,keypoint_number_in front_ego_car, time_taken_discriptor, matched_keypoints"<<endl;;

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results
	int Experiment_number = 1;

    /* MAIN LOOP OVER ALL IMAGES */
	vector<string> detector_type_vector = {"SHITOMASI","HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
	vector<string> decriptor_type_vector = {"BRIEF","BRISK", "ORB", "FREAK", "AKAZE", "SIFT"};
	// extract 2D keypoints from current image
    vector<cv::KeyPoint> keypoints; // create empty feature list for current image
	
	for (auto itr = detector_type_vector.begin(); itr != detector_type_vector.end(); ++itr)
	{
		 string detectorType = *itr;
		for(auto itr2= decriptor_type_vector.begin(); itr2 != decriptor_type_vector.end(); ++itr2)
		{
		    string descriptorType = *itr2; // BRIEF, ORB, FREAK, AKAZE, SIFT
			dataBuffer.clear();
			cout<<endl;
			cout<< " --- Experiment number -- " << Experiment_number << " detector :" << detectorType << " Descriptor :" << descriptorType << endl;
			cout<<endl;
			/*skip any detector rather than AKAZE test with descriptor AKAZE*/
			if(detectorType.compare("AKAZE") != 0 && descriptorType.compare("AKAZE") == 0)
			{
				continue;	
			}
			/*skip detector SIFT test with descriptor ORB as it makes memory error*/
			if(detectorType.compare("SIFT") == 0 && descriptorType .compare("ORB") == 0)
			{
				continue;	
			}
			Experiment_number++;
			for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
			{
				double time;
				/* LOAD IMAGE INTO BUFFER */
				keypoints.clear();

				// assemble filenames for current index
				ostringstream imgNumber;
				imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
				string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

				// load image from file and convert to grayscale
				cv::Mat img, imgGray;
				img = cv::imread(imgFullFilename);
				cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

				//// STUDENT ASSIGNMENT
				//// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

				// push image into data frame buffer
				DataFrame frame;
				frame.cameraImg = imgGray;
				dataBuffer.push_back(frame);

				if(dataBuffer.size() > dataBufferSize)
				{
					dataBuffer.erase(dataBuffer.begin());
				}

				//// EOF STUDENT ASSIGNMENT
				cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
				cout << "The data buffer size " << dataBuffer.size()<<endl;
				/* DETECT IMAGE KEYPOINTS */

				myfile<<imgIndex<<","<< detectorType <<","<< descriptorType <<",";


				//// STUDENT ASSIGNMENT
				//// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
				//// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

				if (detectorType.compare("SHITOMASI") == 0)
				{
					time = detKeypointsShiTomasi(keypoints, imgGray, false);
				}
				else
				{
					time = detKeypointsModern(keypoints,imgGray, detectorType,false);
				}
				
				myfile<<keypoints.size()<<","<< time <<",";
				//// EOF STUDENT ASSIGNMENT

				//// STUDENT ASSIGNMENT
				//// TASK MP.3 -> only keep keypoints on the preceding vehicle

				// only keep keypoints on the preceding vehicle
				bool bFocusOnVehicle = true;
				cv::Rect vehicleRect(535, 180, 180, 150);
				if (bFocusOnVehicle)
				{
					for (auto itr = keypoints.begin(); itr != keypoints.end(); )
					{
						if(vehicleRect.contains(itr->pt) == false)
						{
							itr = keypoints.erase(itr);
						}
						else
						{
							++itr;
						}
					}
				}
				myfile<<keypoints.size()<<",";

				//// EOF STUDENT ASSIGNMENT

				// optional : limit number of keypoints (helpful for debugging and learning)
				bool bLimitKpts = false;
				if (bLimitKpts)
				{
					int maxKeypoints = 50;

					if (detectorType.compare("SHITOMASI") == 0)
					{ // there is no response info, so keep the first 50 as they are sorted in descending quality order
						keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
					}
					cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
					cout << " NOTE: Keypoints have been limited!" << endl;
				}

				// push keypoints and descriptor for current frame to end of data buffer
				(dataBuffer.end() - 1)->keypoints = keypoints;
				cout << "#2 : DETECT KEYPOINTS done" << endl;

				/* EXTRACT KEYPOINT DESCRIPTORS */

				//// STUDENT ASSIGNMENT
				//// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
				//// -> BRIEF, ORB, FREAK, AKAZE, SIFT

				cv::Mat descriptors;
				time = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
				myfile<<time<<",";
				//// EOF STUDENT ASSIGNMENT

				// push descriptors for current frame to end of data buffer
				(dataBuffer.end() - 1)->descriptors = descriptors;

				cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

				if (dataBuffer.size() > 1) // wait until at least two images have been processed
				{

					/* MATCH KEYPOINT DESCRIPTORS */

					vector<cv::DMatch> matches;
					string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
					string descriptorType_M = "DES_BINARY"; // DES_BINARY, DES_HOG
					string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
					
					if((descriptorType.compare("SIFT") == 0)  || (descriptorType.compare("BRIEF") == 0) )
					{
						matcherType = "MAT_BF";
						descriptorType_M = "DES_HOG"; // DES_BINARY, DES_HOG
					}

					//// STUDENT ASSIGNMENT
					//// TASK MP.5 -> add FLANN matching in file matching2D.cpp
					//// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

					matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
									 (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
									 matches, descriptorType_M, matcherType, selectorType);

					//// EOF STUDENT ASSIGNMENT

					// store matches in current data frame
					(dataBuffer.end() - 1)->kptMatches = matches;

					cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
					myfile<<matches.size()<<endl;

					// visualize matches between current and previous image
					bVis = true;
					if (bVis)
					{
						cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
						cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
										(dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
										matches, matchImg,
										cv::Scalar::all(-1), cv::Scalar::all(-1),
										vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

						string windowName = "Matching keypoints between two camera images";
						windowName += " test done with " + descriptorType + " and detector " + detectorType;
						windowName += " sequence no " + to_string(imgIndex);
						windowName += " .jpg";
						//cv::namedWindow(windowName, 7);
						//cv::imshow(windowName, matchImg);
						cv::imwrite("./output/"+windowName, matchImg);
						//cout << "Press key to continue to next image" << endl;
						cv::waitKey(0); // wait for key to be pressed
					}
					bVis = false;
				}
				else
				{
					myfile<<"0"<<endl;
				}
			}
		}

    } // eof loop over all images

    return 0;
}
