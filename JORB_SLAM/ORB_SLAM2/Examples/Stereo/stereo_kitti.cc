/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <unistd.h>
#include <stdlib.h>  

#include <opencv2/core/core.hpp>

#include <System.h>
#include <Server.h>
using namespace std;

constexpr size_t seq_len = 300; //Should be the index of the last element in the sequence
constexpr int seq_A_start = 0;
constexpr int seq_A_end = 150;
constexpr int seq_B_start = 75;
constexpr int seq_B_end = 225;
constexpr int seq_C_start = 150;
constexpr int seq_C_end = 300;
constexpr double fraction_to_remove = 0.15;


//NOTE: Using either failure option decreases the size of output, remove the correct lines in the ground truth to be accurate
int botToFail = 0; //0 = botA, 1 = botB, 2 = botC (this bot will not process images for a time)
bool randomFailure = false; //Randomly fail on botToFail
bool consistentFailure = true; //Fail in consistent frames on botToFail

bool using_three_bots = true;

size_t num_failure_frames = 10; //Number of frames failed
vector<int> failureFrames1;
vector<int> failureFrames2;
vector<int> failureFrames3;

void LoadImages(const string &strPathToSequence, 
                vector<string> &vstrImageLeftSetA, vector<string> &vstrImageRightSetA,
                vector<string> &vstrImageLeftSetB, vector<string> &vstrImageRightSetB,
                vector<double> &vTimestampsA, vector<double> &vTimestampsB);

void LoadImages(const string &strPathToSequence, 
                vector<string> &vstrImageLeftSetA, vector<string> &vstrImageRightSetA,
                vector<string> &vstrImageLeftSetB, vector<string> &vstrImageRightSetB,
		vector<string> &vstrImageLeftSetC, vector<string> &vstrImageRightSetC,
                vector<double> &vTimestampsA, vector<double> &vTimestampsB, vector<double> &vTimestampsC);

void generateFailureFramesRandom(vector<int> &failureFrames, int min, int max, double fractionToFail);

bool isFrameFailure(int id, int botId);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence path_to_server_settings" << endl;
        return 1;
    }

    generateFailureFramesRandom(failureFrames1, seq_A_start, seq_A_end, fraction_to_remove);
    generateFailureFramesRandom(failureFrames2, seq_B_start, seq_B_end, fraction_to_remove);
    generateFailureFramesRandom(failureFrames3, seq_C_start, seq_C_end, fraction_to_remove);

    // Retrieve paths to images
    vector<string> vstrImageLeftSetA;
    vector<string> vstrImageRightSetA;
    vector<string> vstrImageLeftSetB;
    vector<string> vstrImageRightSetB;
    vector<string> vstrImageLeftSetC;
    vector<string> vstrImageRightSetC;
    vector<double> vTimestampsA;
    vector<double> vTimestampsB;
    vector<double> vTimestampsC;
    if (using_three_bots)
    {
	LoadImages(string(argv[3]), vstrImageLeftSetA, vstrImageRightSetA, vstrImageLeftSetB, vstrImageRightSetB, vstrImageLeftSetC, vstrImageRightSetC, vTimestampsA, vTimestampsB, vTimestampsC);
    }
    else
    {
    	LoadImages(string(argv[3]), vstrImageLeftSetA, vstrImageRightSetA, vstrImageLeftSetB, vstrImageRightSetB, vTimestampsA, vTimestampsB);
    }
    const int nImages = vstrImageLeftSetA.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::shared_ptr<ORB_SLAM2::System> SLAM1(new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::STEREO, string("SLAM1"), false));
    std::shared_ptr<ORB_SLAM2::System> SLAM2(new ORB_SLAM2::System(argv[1],argv[2], ORB_SLAM2::System::STEREO, string("SLAM2"), false));
    std::shared_ptr<ORB_SLAM2::System> SLAM3(new ORB_SLAM2::System(argv[1],argv[2], ORB_SLAM2::System::STEREO, string("SLAM3"), false));

    std::shared_ptr<ORB_SLAM2::Server> server(new ORB_SLAM2::Server(argv[4], argv[1]));

    server->RegisterClient(SLAM1.get());
    server->RegisterClient(SLAM2.get());
    if (using_three_bots)
    {
	server->RegisterClient(SLAM3.get());
    }


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;   

    // Main loop
    cv::Mat imLeftA, imRightA, imLeftB, imRightB, imLeftC, imRightC;
    for(size_t ni = 0; ni < seq_len; ni++)
    {
        // Load images from set A
        if(ni < vstrImageLeftSetA.size()) 
        {
            imLeftA = cv::imread(vstrImageLeftSetA[ni],CV_LOAD_IMAGE_UNCHANGED);
            imRightA = cv::imread(vstrImageRightSetA[ni],CV_LOAD_IMAGE_UNCHANGED);
            if(imLeftA.empty())
            {
                cerr << endl << "Failed to load image from set A at:  " << ni << " file: " << string(vstrImageLeftSetA[ni]) << "." << endl;
                return 1;
            }
        }

        // Load images for set b
        if(ni < vstrImageLeftSetB.size()) 
        {
            imLeftB = cv::imread(vstrImageLeftSetB[ni], CV_LOAD_IMAGE_UNCHANGED);
            imRightB = cv::imread(vstrImageRightSetB[ni], CV_LOAD_IMAGE_UNCHANGED);
            if (imLeftB.empty())
            {
                cerr << endl << "Failed to load image from set B at: " << ni << " file: " << string(vstrImageLeftSetB[ni]) << "."  << endl;
                return 1;
            }
        }

	if (using_three_bots && ni < vstrImageLeftSetC.size())
	{
   	    imLeftC = cv::imread(vstrImageLeftSetC[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRightC = cv::imread(vstrImageRightSetC[ni], CV_LOAD_IMAGE_UNCHANGED);
        if (imLeftC.empty())
        {
            cerr << endl << "Failed to load image from set C at: " << ni << " file: " << string(vstrImageLeftSetC[ni]) << "."  << endl;
            return 1;
        }

	}



        if(ni >= vstrImageLeftSetB.size() && ni > vstrImageLeftSetA.size() && (!using_three_bots || ni > vstrImageLeftSetC.size())) {
            std::cout << "Finished all sequences, exiting." << std::endl;
            break;
        };


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Pass the images to the SLAM system
        if(ni < vstrImageLeftSetA.size() && !isFrameFailure(ni+seq_A_start, 0)) 
        {
            SLAM1->TrackStereo(imLeftA,imRightA,vTimestampsA[ni]);
        }
        if(ni < vstrImageLeftSetB.size() && !isFrameFailure(ni+seq_B_start, 1)) 
        {
            SLAM2->TrackStereo(imLeftB,imRightB,vTimestampsB[ni]);
        }
        if (using_three_bots && ni < vstrImageLeftSetC.size() && !isFrameFailure(ni+seq_C_start, 2))
        {
            SLAM3->TrackStereo(imLeftC,imRightC,vTimestampsC[ni]);
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double TA=0,TB=0,TC=0;
        if(ni<nImages-1)
        {
            TA = vTimestampsA[ni+1]-vTimestampsA[ni];
            TB = vTimestampsB[ni+1]-vTimestampsB[ni];
            if (using_three_bots)
            {
                TC = vTimestampsC[ni+1]-vTimestampsC[ni];
            }
        }
        else if(ni>0)
        {
            TA = vTimestampsA[ni]-vTimestampsA[ni-1];
            TB = vTimestampsB[ni]-vTimestampsB[ni-1];
            if (using_three_bots)
            {
                TC = vTimestampsC[ni]-vTimestampsC[ni-1];
            }
        }

        if(ttrack<TA || ttrack<TB)
        {
            double T = TA > TB ? TA : TB;
            // usleep((T-ttrack)*1e6);
        }
    }

    // Wait for the clients to finish processing
    std::this_thread::sleep_for(5s);

    // Disable local mapping and loop closing threads. Keeps viewer alive
    SLAM1->ActivateLocalizationMode();
    SLAM2->ActivateLocalizationMode();
    if (using_three_bots)
    {
    	SLAM3->ActivateLocalizationMode();
    }

    SLAM1->SaveTrajectoryKITTI("CameraTrajectoryA-partial.txt");
    SLAM2->SaveTrajectoryKITTI("CameraTrajectoryB-partial.txt");
    SLAM3->SaveTrajectoryKITTI("CameraTrajectoryC-partial.txt");
    std::cout << "Syncing maps..." << std::endl;

    server->Run();    

    std::cout << "Finished syncing maps." << std::endl;

    std::cout << "[Press enter to shutdown the system]" << std::endl;
    std::cin.ignore();

    // Stop all threads
    server->Shutdown();
    SLAM1->Shutdown();
    SLAM2->Shutdown();
    if (using_three_bots)
    {
    	SLAM3->Shutdown();
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM1->SaveTrajectoryKITTI("CameraTrajectoryA.txt");
    SLAM2->SaveTrajectoryKITTI("CameraTrajectoryB.txt");
    SLAM3->SaveTrajectoryKITTI("CameraTrajectoryC.txt");

    return 0;
}

bool isFrameFailure(int id, int botId)
{
    switch (botId)
    {
        case 0:
            for (int i = 0; i < failureFrames1.size(); i++)
            {
                if (id == failureFrames1[i])
                {
                    return true;
                }
            }
            break;
        case 1:
            for (int i = 0; i < failureFrames2.size(); i++)
            {
                if (id == failureFrames2[i])
                {
                    return true;
                }
            }
            break;
        case 2:
            for (int i = 0; i < failureFrames3.size(); i++)
            {
                if (id == failureFrames3[i])
                {
                    return true;
                }
            }
            break;
    }
    return false;
}

void generateFailureFramesRandom(vector<int> &failureFrames, int min, int max, double fractionToFail)
{
    srand(100);
    int numFail = (max-min)*fractionToFail;
    cout << "Numfail=" << numFail;
    bool uniqueFound = false;
    int val;
    for (int i = 0; i < numFail; i++)
    {
        while (!uniqueFound)
        {
            uniqueFound = true;
            val = (rand() % (max-min)) + min;
            for (int j = 0; j < failureFrames.size(); j++)
            {
                if (val == failureFrames[j])
                {
                    uniqueFound = false;
                    break;
                }
            }
        }   
        uniqueFound = false;
        failureFrames.push_back(val);
    }
}

void LoadImages(const string &strPathToSequence, 
                vector<string> &vstrImageLeftSetA, vector<string> &vstrImageRightSetA,
                vector<string> &vstrImageLeftSetB, vector<string> &vstrImageRightSetB,
                vector<double> &vTimestampsA, vector<double> &vTimestampsB)
{

    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    int counter = 0;
    fTimes.open(strPathTimeFile.c_str());

    double seq_A_start_time;
    double seq_B_start_time;

    size_t seq_A_size = 0;
    size_t seq_B_size = 0;

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            if (counter >= seq_A_start && counter < seq_A_end) 
            {
                if(counter == seq_A_start) seq_A_start_time = t;
                vTimestampsA.push_back(t - seq_A_start_time);
                seq_A_size++;
            }
            if (counter >= seq_B_start && counter < seq_B_end) 
            {
                if(counter == seq_B_start) seq_B_start_time = t;
                vTimestampsB.push_back(t - seq_B_start_time);
                seq_B_size++;
            }
            counter++;
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    vstrImageLeftSetA.resize(seq_A_size);
    vstrImageRightSetA.resize(seq_A_size);
    vstrImageLeftSetB.resize(seq_B_size);
    vstrImageRightSetB.resize(seq_B_size);

    for(size_t i = 0; i < seq_len; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        if(i >= seq_A_start && i < seq_A_end)
        {
			vstrImageLeftSetA[i - seq_A_start] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetA[i -seq_A_start] = strPrefixRight + ss.str() + ".png";
		}
        if(i >= seq_B_start && i < seq_B_end)
		{
			vstrImageLeftSetB[i - seq_B_start] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetB[i - seq_B_start] = strPrefixRight + ss.str() + ".png";
		}
    }
}

void LoadImages(const string &strPathToSequence, 
                vector<string> &vstrImageLeftSetA, vector<string> &vstrImageRightSetA,
                vector<string> &vstrImageLeftSetB, vector<string> &vstrImageRightSetB,
		vector<string> &vstrImageLeftSetC, vector<string> &vstrImageRightSetC,
                vector<double> &vTimestampsA, vector<double> &vTimestampsB, vector<double> &vTimestampsC)
{

    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    int counter = 0;
    fTimes.open(strPathTimeFile.c_str());

    double seq_A_start_time;
    double seq_B_start_time;
    double seq_C_start_time;

    size_t seq_A_size = 0;
    size_t seq_B_size = 0;
    size_t seq_C_size = 0;


    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            if (counter >= seq_A_start && counter < seq_A_end) 
            {
                if(counter == seq_A_start) seq_A_start_time = t;
                vTimestampsA.push_back(t - seq_A_start_time);
                seq_A_size++;
            }
            if (counter >= seq_B_start && counter < seq_B_end) 
            {
                if(counter == seq_B_start) seq_B_start_time = t;
                vTimestampsB.push_back(t - seq_B_start_time);
                seq_B_size++;
            }
	    if (counter >= seq_C_start && counter < seq_C_end) 
            {
                if(counter == seq_C_start) seq_C_start_time = t;
                vTimestampsC.push_back(t - seq_C_start_time);
                seq_C_size++;
            }
            counter++;
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    vstrImageLeftSetA.resize(seq_A_size);
    vstrImageRightSetA.resize(seq_A_size);
    vstrImageLeftSetB.resize(seq_B_size);
    vstrImageRightSetB.resize(seq_B_size);
    vstrImageLeftSetC.resize(seq_C_size);
    vstrImageRightSetC.resize(seq_C_size);

    for(size_t i = 0; i < seq_len; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        if(i >= seq_A_start && i < seq_A_end)
        {
			vstrImageLeftSetA[i - seq_A_start] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetA[i -seq_A_start] = strPrefixRight + ss.str() + ".png";
		}
        if(i >= seq_B_start && i < seq_B_end)
		{
			vstrImageLeftSetB[i - seq_B_start] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetB[i - seq_B_start] = strPrefixRight + ss.str() + ".png";
		}
	    if(i >= seq_C_start && i < seq_C_end)
		{
			vstrImageLeftSetC[i - seq_C_start] = strPrefixLeft + ss.str() + ".png";
			vstrImageRightSetC[i - seq_C_start] = strPrefixRight + ss.str() + ".png";
		}
    }
}

