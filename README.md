# LEGO-LOAM
LEGO-LOAM built in ubuntu20.04 and ros-noetic of course path save
# This project has been built in our environment （as mentioned above)
# LEGO-LOAM is an updated version of A-LOAM,it has more constraints and uses graph optimization
# for more details，you can read this article：
# LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain 
# You can find it in this link：DOI: 10.1109/IROS.2018.8594299


# Before build it,you should read the "readme.txt" in LeGO-LOAM-master so you can understand how to build the raw algorithm.
# Ensure your C++ version is C++ 11 or higher (Ours C++14)
# Firstly,you should install some libraries as follow to ensure your environment the same as ours:
wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip

cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/

cd ~/Downloads/gtsam-4.0.0-alpha2/

mkdir build && cd build

cmake ..

sudo make install

# install another lib:
sudo apt-get install libparmetis-dev
# We made a lot change as possiable to decrease the potential issues caused by version problems，so you don't have to make more change only if your version is the same as ours.
# install,and build it as:
cd LeGO-LOAM

catkin_make

roslaunch lego_loam run.launch

rosbag play yourbag.bag --clock --topic /velodyne_points


# How to save the path in tum format
# you should chagne thevisualizeGlobalMapThread in mapoptimization.cpp as:
void visualizeGlobalMapThread(){

        ros::Rate rate(0.2);	
        while (ros::ok()){	
            rate.sleep();	    
            publishGlobalMap();	    
        }	
        // save final point cloud
        std::cout << "start save final point cloud" << std::endl;
        std::cout << "===========================================" << std::endl;
        ofstream f;
        f.open("/home/u/slam_lidar/src/LeGO-LOAM-master/LeGO-LOAM-master/LeGO-LOAM/path_save/path_tum.txt");						
        f << fixed;
        //std::cout << "traj roll" << cloudKeyPoses6D->points[0].roll << std::endl;
        for(size_t i = 0;i < cloudKeyPoses3D->size();i++)
        {
            float cy = cos((cloudKeyPoses6D->points[i].yaw)*0.5);
            float sy = sin((cloudKeyPoses6D->points[i].yaw)*0.5);
            float cr = cos((cloudKeyPoses6D->points[i].roll)*0.5);
            float sr = sin((cloudKeyPoses6D->points[i].roll)*0.5);
            float cp = cos((cloudKeyPoses6D->points[i].pitch)*0.5);
            float sp = sin((cloudKeyPoses6D->points[i].pitch)*0.5); 
            float w = cy * cp * cr + sy * sp * sr;
            float x = cy * cp * sr - sy * sp * cr;
            float y = sy * cp * sr + cy * sp * cr;
            float z = sy * cp * cr - cy * sp * sr;
            //save the traj  
            f << setprecision(6) << (cloudKeyPoses6D->points[i].time-cloudKeyPoses6D->points[0].time) << " " << setprecision(9) << -1*(cloudKeyPoses6D->points[i].x) << " " << //-1*(cloudKeyPoses6D->points[i].y) << " " << -1*(cloudKeyPoses6D->points[i].z) << " " << x << " " << y << " " << z << " " << w << endl;
        } 
        f.close();
}
# close the terminal,it will save the path in:/home/u/slam_lidar/src/LeGO-LOAM-master/LeGO-LOAM-master/LeGO-LOAM/path_save/path_tum.txt
# You should change the path of the result,but ensure you have the **.txt to save the result!!! 

