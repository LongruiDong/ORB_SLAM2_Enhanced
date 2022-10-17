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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include <include/Converter.h>//用来进行数据转换

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {   // ./mono_replica Vocabulary/ORBvoc.txt Examples/Monocular/Replica.yaml dataset/Replica/office0
        cerr << endl << "Usage: ./mono_replica path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    string ftrack = "FrameTrack.txt";
    ofstream f;
    f.open(ftrack.c_str());
    f << fixed;
    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cv::Mat T_track = SLAM.TrackMonocular(im,tframe); // Tcw
        if(T_track.empty())//合法性检查
        {
            cerr << endl << "WARNING: Empty track pose at: "
                 << ni << endl;
            // return 1;
        }

        if(!T_track.empty())//合法性检查
        {
            // cv::Mat Ttrack = ORB_SLAM2::Converter::InverseMat(T_track.clone());//取逆得到 Twc
            cv::Mat Rwc = T_track.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*T_track.rowRange(0,3).col(3);
            vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

            f << setprecision(6) << tframe << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
    // Stop all threads
    SLAM.Shutdown();

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

    // Save camera trajectory // time x y z i j k w 注意格式
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");  
    // SLAM.SaveTrajectoryMonoTUM("FrameTrajectory.txt"); //也保存所有普通帧轨迹

    // 保存点云到txt
    string ptsfile = "office0_orb_mappts.txt";
    SLAM.SaveMapPoints(ptsfile);

    return 0;
}

// 更改接口来使用replica
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    float fps = 10; // 生成伪时间 但要根据设置的帧率
    float fidx = 0.0; // 初始id
    cout << "pseudo time stamps: " << endl;
    string strPathTimeFile = strPathToSequence + "/traj.txt"; // 用pose 文件 来查看帧数
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        float t;
        getline(fTimes,s);
        if(!s.empty())
        {
            // stringstream ss;
            // ss << s;
            // double t;
            // ss >> t;
            t = fidx*1.0/fps;
            vTimestamps.push_back(t);
            cout << "fid: "<<int(fidx)<<", "<<t<<endl;
            fidx = fidx + 1.0;
        }
    }

    string strPrefixLeft = strPathToSequence + "/results/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i; // dataset/Replica/office0/results/frame000202.jpg
        vstrImageFilenames[i] = strPrefixLeft + "frame" + ss.str() + ".jpg";
    }
}
