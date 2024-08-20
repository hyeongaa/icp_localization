#include "lidar.hpp"


Eigen::Matrix4d performICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score)
{
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaxCorrespondenceDistance(2000); 
    icp.setMaximumIterations(200);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*Final);
    score = icp.getFitnessScore();
    
    
    if(icp.hasConverged() && score < 1.0){
        icp_success = true;
        
        Eigen::Matrix4d matrix = icp.getFinalTransformation().cast<double>();
        
        return matrix;
    }else{
        icp_success = false;
        std::cout<<"ICP not converged the score is "<<score<<std::endl;
        Eigen::Matrix4d matrix_failed = Eigen::Matrix4d::Identity();
        return matrix_failed;
    }
}



Eigen::Matrix4d performGICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score)
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    
    gicp.setMaxCorrespondenceDistance(2000);
    gicp.setMaximumIterations(200);
    gicp.setTransformationEpsilon(0.0001);
    gicp.setEuclideanFitnessEpsilon(0.0001);
    gicp.setRANSACIterations(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>());
    gicp.align(*Final);
    score = gicp.getFitnessScore();

    if(gicp.hasConverged() && score < 2.0){
        icp_success = true;
        Eigen::Matrix4d matrix = gicp.getFinalTransformation().cast<double>();
        return matrix;
    }else{
        icp_success = false;
        std::cout << "GICP did not converge, the score is " << score << std::endl;
        Eigen::Matrix4d matrix_failed = Eigen::Matrix4d::Identity();
        return matrix_failed;
    }
}


Eigen::Matrix4d performNDT(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score)
{
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setInputSource(source_cloud);
    ndt.setInputTarget(target_cloud);
    
    // Parameters
    ndt.setResolution(5.0); // Adjust resolution for balance between speed and accuracy
    ndt.setStepSize(0.2);    // Control the step size for each iteration
    ndt.setTransformationEpsilon(1e-5);
    ndt.setMaximumIterations(200);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>());
    ndt.align(*Final);
    score = ndt.getFitnessScore();

    if(ndt.hasConverged() && score < 2.0){
        icp_success = true;
        Eigen::Matrix4d matrix = ndt.getFinalTransformation().cast<double>();
        return matrix;
    }else{
        icp_success = false;
        std::cout << "NDT did not converge, the score is " << score << std::endl;
        Eigen::Matrix4d matrix_failed = Eigen::Matrix4d::Identity();
        return matrix_failed;
    }
}



double estimate_initial_position(bool& icp_success, Eigen::Matrix4d& icp_transformation, int cnt)
{   
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_cloud = make_cloud_submap(0,cnt*500);
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapcopy(new pcl::PointCloud<pcl::PointXYZI>());

    *mapcopy = *map_cloud;


    voxelize_pcd(aggregated_cloud);
    voxelize_map(mapcopy);

    double score = 100.0;
    bool icp_result= false;


    int size = aggregated_cloud->points.size();

    update_icp_trans.lock();
    for(int i=0; i<size;i++)
    {
        Transformation_points(&aggregated_cloud->points[i],&aggregated_cloud->points[i], icp_transformation_result);
    }
    update_icp_trans.unlock();
    
     
    sensor_msgs::PointCloud2 curr_msgs;
    pcl::toROSMsg(*aggregated_cloud, curr_msgs);
    curr_msgs.header.frame_id = map_frame;       
    pubinit_cloud.publish(curr_msgs);    

    
    icp_transformation = performGICP(aggregated_cloud, mapcopy, icp_result, score);

    /*if(!icp_result && changed == false)
    {   
        Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
        temp.block<3,3>(0,0)<< 0,-1,0,
               1,0,0,
               0,0,1;

        update_icp_trans.lock(); 
        Eigen::Matrix4d pure_rot = icp_transformation_result*temp*icp_transformation_result.inverse();
        update_icp_trans.unlock(); 

        for(int i=0; i<4;i++)
        {

            for(int i=0; i<size;i++)
            {
                Transformation_points(&aggregated_cloud->points[i],&aggregated_cloud->points[i], pure_rot);
            }

            sensor_msgs::PointCloud2 curr_msgs;
            pcl::toROSMsg(*aggregated_cloud, curr_msgs);
            curr_msgs.header.frame_id = map_frame;       
            pubinit_cloud.publish(curr_msgs);   

            icp_transformation = performGICP(aggregated_cloud, mapcopy, icp_result, score);
            
            if(icp_result){
                update_icp_trans.lock(); 
                icp_transformation_result = pure_rot * icp_transformation_result;
                update_icp_trans.unlock();
                break;
            }
        }                

    }*/

    if (icp_result)
    {
        
        int size = aggregated_cloud->points.size();
        pcl::PointCloud<pcl::PointXYZI>::Ptr changed_Cloud(new pcl::PointCloud<pcl::PointXYZI>(size,1));

        for(int i=0; i<size;i++)
        {
            Transformation_points(&aggregated_cloud->points[i], &changed_Cloud->points[i], icp_transformation);
        }
        
        
        update_icp_trans.lock();
        icp_transformation = icp_transformation * icp_transformation_result;
        changed =true;
        update_icp_trans.unlock();



        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = map_frame;
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = icp_transformation(0, 3);
        odom.pose.pose.position.y = icp_transformation(1, 3);
        odom.pose.pose.position.z = icp_transformation(2, 3);

        Eigen::Matrix3d rotation = icp_transformation.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation);
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();
        odom.pose.pose.orientation.w = quaternion.w();


        std::cout<<"ICP successed: Inital point: "<<odom.pose.pose.position.x<<" "<<odom.pose.pose.position.y<<" "<<odom.pose.pose.position.z<<" "<<std::endl;

        sensor_msgs::PointCloud2 chan_msgs;
        pcl::toROSMsg(*changed_Cloud, chan_msgs);
        chan_msgs.header.frame_id = map_frame;

        pubchange.publish(chan_msgs);
        pubinitodom.publish(odom);

    }
    else
    {
        ROS_WARN("ICP did not converge.");
        double score = 200.0;
        changed = false;
    }
    
    
    icp_success = icp_result;

    return score;

}



pcl::PointCloud<pcl::PointXYZI>::Ptr make_cloud_submap(int index, int submap_size)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Matrix4d matrix;
    for(int i=-submap_size; i<=submap_size; i++)
    {
        int curr = index + i;
        if(curr<0 || curr>=frames.size())
            continue;
        
        *pointcloud += *frames.at(curr).lidar_cloud;
    }


    return pointcloud;
}

void voxelize_pcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_pcd;
    voxel_pcd.setLeafSize(scan_voxel_size,scan_voxel_size,scan_voxel_size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr before_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *before_cloud = *cloud;

    voxel_pcd.setInputCloud(before_cloud);
    voxel_pcd.filter(*cloud);

    return;
}

void voxelize_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_pcd;
    voxel_pcd.setLeafSize(map_voxel_size, map_voxel_size, map_voxel_size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr before_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *before_cloud = *cloud;

    voxel_pcd.setInputCloud(before_cloud);
    voxel_pcd.filter(*cloud);

    return;
}

void voxelize_entire_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_pcd;
    voxel_pcd.setLeafSize(10.0,10.0,10.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr before_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *before_cloud = *cloud;

    voxel_pcd.setInputCloud(before_cloud);
    voxel_pcd.filter(*cloud);

    return;
}
