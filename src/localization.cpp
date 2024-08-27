#include "localization.hpp"

double localization(bool& icp_success, Eigen::Matrix4d& icp_transformation, int cnt)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_cloud = make_cloud_submap(0,cnt*500);
    
    double score = 100.0;
    bool icp_result= false;

    //initialization -> only first
    if(!initialized)
    {
        std::cout<<"Initailize Starts!"<<std::endl;
        bool init_suc =false;
        pcl::PointCloud<pcl::PointXYZI>::Ptr BEV_scan_pcd(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr BEV_map(new pcl::PointCloud<pcl::PointXYZI>());

        *BEV_scan_pcd = *aggregated_cloud;
        *BEV_map = *map_cloud;

        voxelize_multi_resolution(BEV_scan_pcd, BEV_voxel_size);
        voxelize_multi_resolution(BEV_map, BEV_voxel_size);

        //map scan, rotate ...
        Eigen::Matrix4d temp = initialize_icp(BEV_scan_pcd, BEV_map, init_suc, map_div_points);

        if(init_suc)
        {   
            update_icp_trans.lock();
            icp_transformation_result = temp;
            update_icp_trans.unlock();
            
            Eigen::Matrix4d result = rotated_icp(BEV_scan_pcd, BEV_map, init_suc, temp, 4);
            
            if(!init_suc) 
            {
                initialized = false;
                return 200;
            }    

            update_icp_trans.lock();
            icp_transformation_result = result;
            update_icp_trans.unlock();

        }else{
            return 200;
        }
    }

    //for fail 5 times consequently

    if(failed_count ==5 || failed_bool)
    {
        std::cout<<"Failed ICP Starts: "<<std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr failed_scan(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr failed_map(new pcl::PointCloud<pcl::PointXYZI>());

        *failed_scan = *aggregated_cloud;
        *failed_map = *map_cloud;

        voxelize_multi_resolution(failed_scan,scan_voxel_size);
        voxelize_multi_resolution(failed_map,scan_voxel_size);


        bool succ = false;
        Eigen::Matrix4d failed_result = rotated_icp(failed_scan, failed_map, succ, icp_transformation_result,8);
        if(succ)
        {
            update_icp_trans.lock();
            icp_transformation_result = failed_result;
            update_icp_trans.unlock();
        }else{
            initialized = false;
            BEV_voxel_size = 1.0;
        }

        failed_count=0;
        failed_bool = false;
        return 200;
    }

///////////////////// now localization start /// Main function

    int size = aggregated_cloud->points.size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    *scan = *aggregated_cloud;
    *map = *map_cloud;
    voxelize_multi_resolution(scan,scan_voxel_size);
    voxelize_multi_resolution(map, map_voxel_size);
    
    update_icp_trans.lock();
    for(int i=0; i<size;i++)
    {
        Transformation_points(&scan->points[i], &scan->points[i], icp_transformation_result);
    }
    update_icp_trans.unlock();

    sensor_msgs::PointCloud2 curr_msgs;
    pcl::toROSMsg(*scan, curr_msgs);
    curr_msgs.header.frame_id = map_frame;       
    pubinit_cloud.publish(curr_msgs); 


    icp_transformation = performGICP(scan, map, icp_result, score);

    if (icp_result)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr changed_Cloud(new pcl::PointCloud<pcl::PointXYZI>(size,1));
        

        for(int i=0; i<size;i++)
        {
            Transformation_points(&scan->points[i], &changed_Cloud->points[i], icp_transformation);
        }
        
        update_icp_trans.lock();
        icp_transformation_result = icp_transformation * icp_transformation_result;
        update_icp_trans.unlock();  

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = map_frame;
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = icp_transformation_result(0, 3);
        odom.pose.pose.position.y = icp_transformation_result(1, 3);
        odom.pose.pose.position.z = icp_transformation_result(2, 3);

        Eigen::Matrix3d rotation = icp_transformation_result.block<3, 3>(0, 0);
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
        failed_count =0 ;

    }
    else
    {
        ROS_WARN("ICP did not converge.");
        double score = 200.0;
        failed_count++;
    }



    icp_success = icp_result;

    return score;

}

double localization_scan(bool& icp_success, Eigen::Matrix4d& icp_transformation, int cnt)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_cloud = make_cloud_submap(0,cnt*500);
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapcopy(new pcl::PointCloud<pcl::PointXYZI>());

    *mapcopy = *map_cloud;


    double score = 100.0;
    bool icp_result= false;

    if(!initialized)
    {
        bool init_suc =false;
        pcl::PointCloud<pcl::PointXYZI>::Ptr BEV_scan_pcd(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr BEV_map(new pcl::PointCloud<pcl::PointXYZI>());

        *BEV_scan_pcd = *aggregated_cloud;
        *BEV_map = *map_cloud;

        voxelize_multi_resolution(BEV_scan_pcd, BEV_voxel_size);
        voxelize_multi_resolution(BEV_map, BEV_voxel_size);

        //map scan, rotate ...
        Eigen::Matrix4d temp = initialize_icp(BEV_scan_pcd, BEV_map, init_suc, map_div_points);

        if(init_suc)
        {   
            update_icp_trans.lock();
            icp_transformation_result = temp;
            update_icp_trans.unlock();
            
            Eigen::Matrix4d result = rotated_icp(BEV_scan_pcd, BEV_map, init_suc, temp,4);
            
            if(!init_suc) 
            {
                initialized = false;
                return 200;
            }    

            update_icp_trans.lock();
            icp_transformation_result = result;
            update_icp_trans.unlock();

        }else{
            return 200;
        }

    }



    if(failed_count ==5 || failed_bool)
    {
        voxelize_pcd(aggregated_cloud);
        voxelize_map(mapcopy);
        bool succ = false;
        Eigen::Matrix4d failed_result = rotated_icp(aggregated_cloud, mapcopy, succ, icp_transformation_result,8);
        if(succ)
        {
            update_icp_trans.lock();
            icp_transformation_result = failed_result;
            update_icp_trans.unlock();
        }

        failed_count=0;
        failed_bool = false;
        return 200;
    }



    int size = aggregated_cloud->points.size();
    std::vector<double> multi_resol_sizes = {0.8, 0.4, 0.2};

    for(int i=0; i< multi_resol_sizes.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
        *scan = *aggregated_cloud;
        *map = *map_cloud;
        voxelize_multi_resolution(scan, multi_resol_sizes.at(i));
        voxelize_multi_resolution(map, multi_resol_sizes.at(i));

        update_icp_trans.lock();
        for(int i=0; i<size;i++)
        {
            Transformation_points(&scan->points[i],&scan->points[i], icp_transformation_result);
        }
        update_icp_trans.unlock();


        sensor_msgs::PointCloud2 curr_msgs;
        pcl::toROSMsg(*scan, curr_msgs);
        curr_msgs.header.frame_id = map_frame;       
        pubinit_cloud.publish(curr_msgs); 


        icp_transformation = performGICP(scan, map, icp_result, score);
    }

    



    if (icp_result)
    {
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr changed_Cloud(new pcl::PointCloud<pcl::PointXYZI>(size,1));
        
        update_icp_trans.lock();
        icp_transformation = icp_transformation * icp_transformation_result;
        update_icp_trans.unlock();

        for(int i=0; i<size;i++)
        {
            Transformation_points(&aggregated_cloud->points[i], &changed_Cloud->points[i], icp_transformation);
        }
        
        
 



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
        failed_count =0 ;

    }
    else
    {
        ROS_WARN("ICP did not converge.");
        double score = 200.0;
        failed_count++;
    }
    
    
    icp_success = icp_result;

    return score;

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
    }
    
    
    icp_success = icp_result;

    return score;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertToBEV(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr bev_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (const auto& point : cloud->points) {
        pcl::PointXYZ bev_point;
        bev_point.x = point.x;
        bev_point.y = point.y;
        bev_point.z = 0;
        bev_cloud->points.push_back(bev_point);
    }
    
    return bev_cloud;
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
    voxel_pcd.setLeafSize(map_entire_voxel_size,map_entire_voxel_size,map_entire_voxel_size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr before_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *before_cloud = *cloud;

    voxel_pcd.setInputCloud(before_cloud);
    voxel_pcd.filter(*cloud);

    return;
}

void voxelize_multi_resolution(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double size)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_pcd;
    voxel_pcd.setLeafSize(size,size,size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr before_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *before_cloud = *cloud;

    voxel_pcd.setInputCloud(before_cloud);
    voxel_pcd.filter(*cloud);

    return;
}