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
#include <chrono>
#include <atomic>
#include <thread>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>

#include <System.h>

void stop_falg_detection();

// A flag to indicate whether a key had been pressed.
std::atomic_bool stop_flag(false);

int main(int argc, char **argv) try {

    if(argc != 3){
        cerr << endl << "Usage: ./rgbd_realsense path_to_vocabulary path_to_settings" << endl;
        return EXIT_SUCCESS;
    }

    std::cout << "Querying Realsense device info..." << std::endl;

    // Create librealsense context for managing devices
    rs2::context ctx;
    auto devs = ctx.query_devices();  // Get device list
    int device_num = devs.size();
    std::cout << "Device number: " << device_num << std::endl; // Device amount

    // Query the info of first device
    rs2::device dev = devs[0];  // If no device conneted, a rs2::error exception will be raised
    // Device serial number (different for each device, can be used for searching device when having mutiple devices)
    std::cout << "Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

    rs2::config cfg;
    // Default it will config all the devices，you can specify the device index you want to config (query by serial number)
    // Config color stream: 640*480, frame format: BGR, FPS: 30
    cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_RGB8, 30);  // BGR8 correspond to CV_8UC3 in OpenCV
    // Config depth stream: 640*480, frame format: Z16, FPS: 30
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30); // Z16 corresponds to CV_16U in OpenCV

    std::cout << "Config RGB frame format to 8-channal RGB" << std::endl;
    std::cout << "Config RGB and depth FPS to 30" << std::endl;

    rs2::pipeline pipe;
    pipe.start(cfg);

    // Block program until frames arrive
    rs2::frameset data = pipe.wait_for_frames();

    rs2::depth_frame depth = data.get_depth_frame();
    rs2::video_frame color = data.get_color_frame();

    rs2::stream_profile depth_profile = depth.get_profile();
    rs2::stream_profile color_profile = color.get_profile();

    // Get RGB camera intrinsics
    // Note that the change of config will cause the change of intrinsics
    rs2::video_stream_profile cvsprofile(color_profile);
    rs2::video_stream_profile dvsprofile(depth_profile);
    rs2_intrinsics color_intrinsics = cvsprofile.get_intrinsics();
    rs2_intrinsics depth_intrinsics = dvsprofile.get_intrinsics();

    const int color_width = color_intrinsics.width;
    const int color_height = color_intrinsics.height;
    const int depth_width = depth_intrinsics.width;
    const int depth_height = depth_intrinsics.height;

    std::cout << "RGB Frame width: " << color_width << std::endl;
    std::cout << "RGB Frame height: " << color_height << std::endl;
    std::cout << "Depth Frame width: " << depth_width << std::endl;
    std::cout << "Depth Frame height: " << depth_height << std::endl;
    std::cout << "RGB camera intrinsics:" << std::endl;
    std::cout << "fx: " << color_intrinsics.fx << std::endl;
    std::cout << "fy: " << color_intrinsics.fy << std::endl;
    std::cout << "cx: " << color_intrinsics.ppx << std::endl;
    std::cout << "cy: " << color_intrinsics.ppy << std::endl;
    std::cout << "RGB camera distortion coeffs:" << std::endl;
    std::cout << "k1: " << color_intrinsics.coeffs[0] << std::endl;
    std::cout << "k2: " << color_intrinsics.coeffs[1] << std::endl;
    std::cout << "p1: " << color_intrinsics.coeffs[2] << std::endl;
    std::cout << "p2: " << color_intrinsics.coeffs[3] << std::endl;
    std::cout << "k3: " << color_intrinsics.coeffs[4] << std::endl;
    //std::cout << "RGB camera distortion model: " << color_intrinsics.model << std::endl;

    std::cout << "* Please adjust the parameters in config file accordingly *" << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vtimes_track;

    std::thread stop_detect_thread = std::thread(stop_falg_detection);

    std::cout << std::endl << "-------" << std::endl;
    std::cout << "Start processing realsense stream ..." << std::endl;
    std::cout << "Use 'e + enter' to end the system" << std::endl;

    while (!stop_flag){
        data = pipe.wait_for_frames();

        depth = data.get_depth_frame();
        color = data.get_color_frame();
        double time_stamp = data.get_timestamp();

        cv::Mat im_D(cv::Size(depth_width, depth_height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat im_RGB(cv::Size(color_width, color_height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(im_RGB,im_D,time_stamp);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        vtimes_track.push_back(ttrack);
    }

    stop_detect_thread.join();

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vtimes_track.begin(),vtimes_track.end());
    float time_total = 0;
    for(size_t i = 0; i < vtimes_track.size(); i++){
        time_total += vtimes_track[i];
    }

    std::cout << "-------" << std::endl << std::endl;
    std::cout << "median tracking time: " << vtimes_track[vtimes_track.size() / 2] << std::endl;
    std::cout << "mean tracking time: " << time_total / vtimes_track.size() << std::endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return EXIT_SUCCESS;

}catch(const rs2::error &e){
    // Capture device exception
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}catch(const std::exception &e){
    std::cerr<<"Other error : " << e.what() << std::endl;
    return EXIT_FAILURE;
}

void stop_falg_detection(){
    char c;
    while (!stop_flag) {
        c = std::getchar();
        if(c == 'e'){
            stop_flag = true;;
        }
    }
}