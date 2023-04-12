#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_object_detection/pointcloudConfig.h>
#include <tf/transform_broadcaster.h>

ros::Publisher cloud_pub;
ros::Publisher marker_pub;
ros::Publisher marker_pub2;
ros::Subscriber sub;
std_msgs::Header msg_header;
sensor_msgs::PointCloud2::ConstPtr input_cloud_data;

void publish(pcl::PointCloud<pcl::PointXYZ> cloud);
void callback(dynamic_object_detection::pointcloudConfig &config, uint32_t level);

int ros_hz = 4;          // 執行頻率
bool fisrt_data = false; // 有資料才演算

// 點雲演算法
class PointCloud
{
private:
    pcl::PCLPointCloud2 pcl_pc2; // 原始資料(!!)
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; // 體速完
    pcl::PassThrough<pcl::PointXYZ> pass;              // 直通完的資料
    pcl::VoxelGrid<pcl::PointXYZ> sor;                 // 體速完的資料
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;// 統計完的資料
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud;
    double detect_range = 1.0; // 直通濾波器範圍
    float leafsize3d = 0.09;   // 體速網格濾波器之範圍(M)
    float eps = 0.1f;          // DBSCAN 的範圍
    int minPts = 10;           // DBSCAN 的距離
    int MeanK=50;
    int MulThresh=10;
    float RadiusSearch=0.04;
    int MinNeighborsInRadius=5;
    std_msgs::Header header;

public:
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;          // 原始處理完的資料
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloud; // 輸出資料
    visualization_msgs::MarkerArray::Ptr marker_array;  // maker data
    pcl::ModelCoefficients::Ptr coefficients; //地面斜線參數
    pcl::PointIndices::Ptr inliers; // 地面點雲
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals ;
    std::list<pcl::Normal> normal_list;
    // 初始化參數
    PointCloud()
    {

        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        outputcloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        marker_array.reset(new visualization_msgs::MarkerArray);
        cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        coefficients.reset(new pcl::ModelCoefficients);
        inliers.reset(new pcl::PointIndices);
        cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr start(const sensor_msgs::PointCloud2::ConstPtr &input_cloud,  std_msgs::Header header_msg)
    {
        outputcloud->clear();
        clusterIndices.clear();
        header = header_msg;
        pcl_conversions::toPCL(*input_cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud); 
        //std::cout << "The shared point cloud contains " << cloud->size() << " points." << std::endl;
        return cloud;  
    }

    void parameter(dynamic_object_detection::pointcloudConfig &config)
    {
        ROS_INFO("Reconfigure Request: %d %f %f %f %d",
               config.ros_hz, config.detect_range, config.leafsize3d, config.eps, config.minPts);
        
        detect_range = config.detect_range; 
        leafsize3d = config.leafsize3d;    
        eps = config.eps;                   
        minPts = config.minPts;           
        MeanK = config.MeanK;
        MulThresh = config.StddevMulThresh;
        RadiusSearch = config.RadiusSearch;
        MinNeighborsInRadius = config.MinNeighborsInRadius;
    }

    // 直通濾波器
    void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        // ROS_INFO("cloud_num1 : %f",cloud_data->makeShared());
        // pass.setInputCloud(cloud_data->makeShared());
        // pass.setFilterFieldName("z");
        // pass.setFilterLimits(0.0, 1.0);
        // pass.filter(*cloud_data);
        pass.setInputCloud(cloud_data->makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-detect_range, detect_range);
        pass.filter(*cloud_data);
        pass.setInputCloud(cloud_data->makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-detect_range, detect_range);
        pass.filter(*cloud_data);
    }

    // 體素濾波器
    void Points_Simplification(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data, bool is_2D)
    {
        double z;
        if (is_2D)
        {
            z = 0.0;
        }
        else
        {
            z = leafsize3d;
        }
        sor.setInputCloud(cloud_data->makeShared());
        sor.setLeafSize(leafsize3d, leafsize3d, z);
        sor.filter(*cloud_data);
    }
    //統計濾波器
    void StatisticalOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
        sor2.setInputCloud(cloud_data->makeShared());
        sor2.setMeanK(MeanK); // Set number of neighbors to be considered for averaging
        sor2.setStddevMulThresh(MulThresh); // Set standard deviation multiplier

        // Filter the cloud
        sor2.filter(*cloud_data);
    }

    //地面演算法(sample consensus)
    void RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
        pcl::SACSegmentation<pcl::PointXYZ> sac;
        sac.setInputCloud(cloud_data);
        sac.setMethodType(pcl::SAC_RANSAC);
        sac.setModelType(pcl::SACMODEL_PLANE);
        sac.setDistanceThreshold(0.01);
        sac.setMaxIterations(1000);
        // sac.setInputNormals(cloud_normals);
        sac.setProbability(0.95);
        sac.segment(*inliers, *coefficients);
    }

    // 移除地面
    void NoGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_data);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_data);
    }
    //顯示地面
    // void Ground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
    //     // Generate random color
    //     printf("hi");
    //     uint8_t r = 255;
    //     uint8_t g = 255;
    //     uint8_t b = 255;
    //     for (std::vector<int>::const_iterator pit = inliers->indices.begin(); pit != inliers->indices.end(); ++pit)
    //     {
    //         pcl::PointXYZRGB colored_point;
    //         colored_point.x = cloud_data->points[*pit].x;
    //         colored_point.y = cloud_data->points[*pit].y;
    //         colored_point.z = cloud_data->points[*pit].z;
    //         colored_point.r = r;
    //         colored_point.g = g;
    //         colored_point.b = b;
    //         outputcloud->points.push_back(colored_point);
    //     }
    //     outputcloud->width = outputcloud->points.size();
    //     outputcloud->height = 1;
    //     outputcloud->is_dense = true;
    // }
    // 映射降維度
    void ProjectInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
        pcl::ModelCoefficients::Ptr mc (new pcl::ModelCoefficients);
        mc->values.resize(4);
        mc->values[0] = mc->values[1] = 0;
        mc->values[2] = 1.0;
        mc->values[3] = 0;      
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE); 
        proj.setInputCloud(cloud_data); 
        proj.setModelCoefficients(mc); 
        // proj.setIndices(inliers); 

        // pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        proj.filter(*cloud_data); 
    }
    
    // PCA
    void PCA( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_data);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.03);
        ne.compute (*cloud_normals);

        // pcl::Normal overall_normal(0, 0, 0);
        // // printf("0 = %f\n",overall_normal.normal_x);
        // for (size_t i = 0; i < cloud_normals->size(); ++i) {
        //     if(cloud_normals->points[i].normal_x!=cloud_normals->points[i].normal_x){
        //         // error++;   
        //         // printf("time %d = %f\n",i,cloud_normals->points[i].normal_x);
        //         continue;
        //     }
        //     overall_normal.normal_x += cloud_normals->points[i].normal_x;
        //     overall_normal.normal_y += cloud_normals->points[i].normal_y;
        //     overall_normal.normal_z += cloud_normals->points[i].normal_z;

        //     // overall_normal.normal_x += abs(cloud_normals->points[i].normal_x);
        //     // overall_normal.normal_y += abs(cloud_normals->points[i].normal_y);
        //     // overall_normal.normal_z += abs(cloud_normals->points[i].normal_z);
        // }

        // if(normal_list.size()>=100){
        //     normal_list.erase(normal_list.begin());  // 刪除頭一筆資料
        // }
        // printf("time 1 = %f\n",overall_normal.normal_x);
        // normal_list.push_back(overall_normal);  // 新增資料到最後面

    }

    //半徑濾波器
    void RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud (cloud_data);  // 将点云输入到滤波器
        outrem.setRadiusSearch (RadiusSearch);  // 设置近邻点搜索半径
        outrem.setMinNeighborsInRadius (MinNeighborsInRadius);  // 设置查询点最小近邻点数
        outrem.filter (*cloud_data);
    }

    // DBSCAN
    void DBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_data->makeShared());
        ec.setClusterTolerance(eps);
        ec.setMinClusterSize(minPts);
        ec.setMaxClusterSize(cloud_data->size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_data->makeShared());
        ec.extract(clusterIndices);
    }

    // 測試並顯示顏色
    void testcolor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        // Iterate through each cluster
        ROS_INFO("passstartnum : %d", clusterIndices.size() );
        for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
        {
            pcl::PointXYZRGB colored_point;
            colored_point.r = rand() % 256;
            colored_point.g = rand() % 256;
            colored_point.b = rand() % 256;
            // ROS_INFO("passstartnum : %d", colored_point.r );
            //  Iterate through each point in the cluster
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                // Set point color
                colored_point.x = cloud_data->points[*pit].x;
                colored_point.y = cloud_data->points[*pit].y;
                colored_point.z = cloud_data->points[*pit].z;

                // Add colored point to new cloud
                outputcloud->points.push_back(colored_point);
            }
        }
        // Set point cloud header
        // ROS_INFO("Output cloud contains %ld points", clusterIndices.size());
        outputcloud->width = outputcloud->points.size();
        outputcloud->height = 1;
        outputcloud->is_dense = true;
    }

    void object_boxes( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        // marker.header.stamp = ros::Time();
        marker.ns = "object_boxes";
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        // Draw bounding boxes for each cluster
        int cluster_id = 0;
        marker_array->markers.clear();
        for (auto indices : clusterIndices)
        {
            drawBoundingBox(marker, cloud_data, pcl::PointIndices::Ptr(new pcl::PointIndices(indices)), cluster_id++);
            marker_array->markers.push_back(marker);
        }
        // ROS_INFO("%d",marker_array->markers.size());
    }

    void drawBoundingBox(visualization_msgs::Marker &marker, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data, pcl::PointIndices::Ptr indices, int id)
    {

         pcl::copyPointCloud(*cloud_data, indices->indices, *cluster_cloud);

        //orintation
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cluster_cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.03);
        ne.compute(*normals);

        // Estimate principal curvatures
        pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
        pc.setInputCloud(cluster_cloud);
        pc.setInputNormals(normals);
        pc.setSearchMethod(tree);
        pc.setRadiusSearch(0.03);
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pcs(new pcl::PointCloud<pcl::PrincipalCurvatures>);
        pc.compute(*pcs);

        // Compute PCA of the point cloud
        Eigen::Matrix3f covariance;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster_cloud, centroid);
        pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

        // Convert eigen_vectors to quaternion
        Eigen::Quaternionf q(eigen_vectors);
        tf::Quaternion orientation(q.x(), q.y(), q.z(), q.w());
        
        //
        Eigen::Vector4f  minPt, maxPt;
        pcl::getMinMax3D(*cluster_cloud, minPt, maxPt);

        int n_points = cluster_cloud->points.size();
        Eigen::MatrixXf points_matrix(n_points, 3);
        Eigen::Vector3f min,max;
        // Define the bounding box dimensions
        min = eigen_vectors * minPt.block<3,1>(0,0);
        max = eigen_vectors * maxPt.block<3,1>(0,0);
        float x_min = min[0];
        float x_max = max[0];
        float y_min = min[1];
        float y_max = max[1];

        // Draw the bounding box
        marker.id = id;
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        // Eigen::Quaternionf quat(Eigen::AngleAxisf(0, (0,0,0)));

        // marker.pose.orientation.x = 0;
        // marker.pose.orientation.y = 0;
        // marker.pose.orientation.z = 0;
        // marker.pose.orientation.w = 1;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = x_max - x_min;
        marker.scale.y = y_max - y_min;
        marker.scale.z = 1;
        marker.color.a = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
    }

    void TransformedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
        Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f target_direction(0.0, 0.0, 1.0);
        Eigen::Quaternionf q;
        q.setFromTwoVectors(plane_normal, target_direction);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = q.toRotationMatrix();

        // 将点云转换到新方向
        // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*cloud_data, *cloud_data, transform);

        // 保存转换后的点云
        // pcl::io::savePLYFile ("transformed_cloud.ply", *transformed_cloud);
    }

    void aaa(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
        pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(10);
        reg.setMaxClusterSize(1000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);  
        reg.setInputCloud(cloud_data);

        // Compute normals for the input cloud
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud_data);
        normalEstimation.setSearchMethod(tree);
        normalEstimation.setKSearch(30);
        normalEstimation.compute(*normals);
        reg.setInputNormals(normals);

        // Extract the clusters from the point cloud
        reg.extract(clusterIndices);
    }
    
};

PointCloud PCM;

// 接收資料
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud)
{
    fisrt_data = true;
    input_cloud_data = input_cloud;
    msg_header = input_cloud->header;
}

// 傳回ROS
void cloudPublish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud, output_cloud);

    output_cloud.header = msg_header;
    cloud_pub.publish(output_cloud);
}
void makerPublish(visualization_msgs::MarkerArray::Ptr markerarray)
{
    // ROS_INFO("%c",markerarray->markers.size());
    marker_pub.publish(*markerarray);
}
void PCAPublish(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals ,std::list<pcl::Normal> normal_list){
    // 將法向量轉換為ROS Marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "point_cloud_normals";
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;



    // printf("2 = %f\n",overall_normal.normal_x);
    geometry_msgs::Point p ,q;
    p.x = q.x = 0;
    p.y = q.y = 0;
    p.z = q.z = 0;

    // for(std::list<pcl::Normal>::iterator it = normal_list.begin(); it != normal_list.end(); ++it){
    //     q.x += abs(it->normal_x);
    //     q.y += abs(it->normal_y);
    //     q.z += abs(it->normal_z);
    // }
    for(std::list<pcl::Normal>::iterator it = normal_list.begin(); it != normal_list.end(); ++it){
        q.x = abs(it->normal_x);
        q.y = abs(it->normal_y);
        q.z = abs(it->normal_z);
    }

    marker.points.push_back(p);
    marker.points.push_back(q);
    marker_pub2.publish(marker); 
}

int main(int argc, char **argv)
{
    // ROS 初始化
    ros::init(argc, argv, "pcl");
    ros::NodeHandle nh;
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, cloudCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/output_cloud", 1);
    marker_pub2 = nh.advertise<visualization_msgs::Marker>("point_cloud_normals", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("object_bounding_boxes", 10000);
    ros::Publisher marker_pub3 = nh.advertise<visualization_msgs::Marker>("output", 1);
    dynamic_reconfigure::Server<dynamic_object_detection::pointcloudConfig> server;
    dynamic_reconfigure::Server<dynamic_object_detection::pointcloudConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_data;

    std::vector<pcl::PointIndices> dbscan_data;

    ros::Rate loop_rate(ros_hz);



    
    while (ros::ok())
    {   
        ros::spinOnce();
        if (fisrt_data)
        {
            
            point_data = PCM.start(input_cloud_data, msg_header);

            PCM.PassThroughFilter(point_data);
            PCM.Points_Simplification(point_data, false);
            // PCM.PCA(point_data);
            // PCAPublish(PCM.cloud_normals,PCM.normal_list);
            
            // std::cout << "The shared point cloud contains " << point_data->size() << " points." << std::endl;
            // PCM.PassThroughFilter(point_data);
            // PCM.Points_Simplification(point_data, false);
            // // PCM.StatisticalOutlier(point_data);

            PCM.RANSAC(point_data);
            PCM.NoGround(point_data);
            
            // PCM.ProjectInliers(point_data);
            // std::cout << "The 1 " << point_data->size() << " points." << std::endl;
            // PCM.RadiusOutlierRemoval(point_data);
            // std::cout << "The 2 " << point_data->size() << " points." << std::endl;
            // PCM.StatisticalOutlier(point_data);

            // PCM.Points_Simplification(point_data, true);
            // PCM.TransformedCloud(point_data);
            PCM.aaa(point_data);
            // PCM.DBSCAN(point_data);
            PCM.testcolor(point_data);
            cloudPublish(PCM.outputcloud);
            PCM.object_boxes(point_data);
            makerPublish(PCM.marker_array);
            // // std::cout << "The shared point cloud contains " << point_data->size() << " points." << std::endl;
        }
        // ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}

// 更改參數
void callback(dynamic_object_detection::pointcloudConfig &config, uint32_t level)
{
    
    PCM.parameter(config);
    if (ros_hz != config.ros_hz){
        ros::Rate loop_rate(config.ros_hz);
    }
    ros_hz = config.ros_hz;
}       