// 親愛的設計師：
// 當初我初次寫下這段程式碼 只有雲千和我和上帝知道他是如何運作的
// 如今只剩下雲千和上帝知道

// 因此假如你試著最佳化程式或是讓他能動
// 問雲千就對了

// 如果雲千也不在世了話
// 你修改這斷程式失敗了話請增加下面計數器的時間
// 作為給下一個人的警訊：
// 總共浪費時間：87

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
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

#include <ctime>
#include <unistd.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_object_detection/pointcloudConfig.h>
#include <tf/transform_broadcaster.h>
#include <pcl/registration/icp.h>

ros::Publisher cloud_pub;
// ros::Publisher marker_pub;
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
    pcl::PCLPointCloud2 pcl_pc2;                        // 原始資料(!!)
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // 體速完
    pcl::PassThrough<pcl::PointXYZ> pass;               // 直通完的資料
    pcl::VoxelGrid<pcl::PointXYZ> sor;                  // 體速完的資料
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2; // 統計完的資料
    double detect_range = 1.0;                          // 直通濾波器範圍
    float leafsize3d = 0.09;                            // 體速網格濾波器3D之範圍(M)
    float leafsize2d = 0.12 ;                           // 體速網格濾波器2D之範圍(M)
    float eps = 0.1f;                                   // DBSCAN 的範圍
    int minPts = 10;                                    // DBSCAN 的距離
    float eps2 = 0.1f;                                  // DBSCAN較大物件 的範圍
    int minPts2 = 10;                                   // DBSCAN較大物件 的距離
    int MeanK = 50;
    int MulThresh = 10;
    float RadiusSearch = 0.04;
    int MinNeighborsInRadius = 2;
    std_msgs::Header header;
    int MAXClusterSize = 20;
    int MinClusterSize = 10;
    int NumberOfNeighbours = 20;
    int KSearch = 20;
    int max_points_per_cluster = 100;

public:
    std::vector<pcl::PointIndices> clusterIndices;
    std::vector<pcl::PointIndices> clusterIndicesBIG[5];
    std::vector<pcl::PointIndices>::iterator BIG[5];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;          // 原始處理完的資料
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloud; // 輸出資料
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloudg; // 輸出資料
    // visualization_msgs::MarkerArray::Ptr marker_array;  // maker data
    visualization_msgs::MarkerArray::Ptr marker_static;  // maker data
    visualization_msgs::MarkerArray::Ptr marker_dynamic;  // maker data
    pcl::ModelCoefficients::Ptr coefficients;           // 地面斜線參數
    pcl::PointIndices::Ptr inliers;                     // 地面點雲
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    std::list<pcl::Normal> normal_list;
    const int mapSize = 6;  // 地图大小（米）
    const float pixelResolution = 0.1;  // 每个像素代表的距离（米）
    const int frameCount = 10;  // 近10帧点云数据
    const int minValidFrames = 6;  // 最小有效帧数
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> slidingWindow;
    int mapWidth;
    int mapHeight;
    std::vector<float> mapData;
    int validFrames = 0;
    std::vector<std::array<double, 8>> frame;
    struct Point {
        double x, y;
    };

    // 初始化參數
    PointCloud()
    {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        outputcloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        outputcloudg.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        // marker_array.reset(new visualization_msgs::MarkerArray);
        marker_static.reset(new visualization_msgs::MarkerArray);
        marker_dynamic.reset(new visualization_msgs::MarkerArray);
        coefficients.reset(new pcl::ModelCoefficients);
        inliers.reset(new pcl::PointIndices);
        cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
        
        mapWidth = static_cast<int>(mapSize / pixelResolution);
        mapHeight = static_cast<int>(mapSize / pixelResolution);
        mapData.resize(mapWidth * mapHeight, -1.0f);
        
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr start(const sensor_msgs::PointCloud2::ConstPtr &input_cloud, std_msgs::Header header_msg)
    {
        frame.clear();
        outputcloud->clear();
        outputcloudg->clear();
        // marker_array->markers.clear();
        marker_static->markers.clear();
        marker_dynamic->markers.clear();
        // clusterIndicesBIG.clear();
        for (int i = 0; i < 5; i++)
        {
            clusterIndicesBIG[i].clear();
            BIG[i] = clusterIndicesBIG[i].begin();
        }

        header = header_msg;
        pcl_conversions::toPCL(*input_cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
        // std::cout << "The shared point cloud contains " << cloud->size() << " points." << std::endl;
        return cloud;
    }

    void parameter(const dynamic_object_detection::pointcloudConfig &config)
    {
        // ROS_INFO("Reconfigure Request: %d %f %f %f %d",
        //        config.ros_hz, config.detect_range, config.leafsize3d, config.eps, config.minPts);

        detect_range = config.detect_range;
        leafsize3d = config.leafsize3d;
        leafsize2d = config.leafsize2d;
        eps = config.DBSCAN_eps;
        minPts = config.DBSCAN_minPts;
        eps2 = config.DBSCAN_eps2;
        minPts2 = config.DBSCAN_minPts2;
        MeanK = config.Statistical_MeanK;
        MulThresh = config.Statistical_Thresh;
        RadiusSearch = config.Radius_RadiusSearch;
        MinNeighborsInRadius = config.Radius_MinNeighborsInRadius;
        MAXClusterSize = config.RGS_MAXClusterSize;
        MinClusterSize = config.RGS_MinClusterSize;
        NumberOfNeighbours = config.RGS_NumberOfNeighbours;
        KSearch = config.RGS_KSearch;
    }

    // 直通濾波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpoint;
        cloudpoint.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(cloud_data->makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-detect_range, detect_range);
        pass.filter(*cloud_data);
        pass.setInputCloud(cloud_data->makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-detect_range, detect_range);
        pass.filter(*cloudpoint);
        return cloudpoint;
    }

    // 體素濾波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr Points_Simplification(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data, bool is_2D)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpoint;
        cloudpoint.reset(new pcl::PointCloud<pcl::PointXYZ>);
        double z,x,y;
        // printf("%lf",leafsize2d);
        if (is_2D)
        {
            z = 0.0;
            x = leafsize3d;
            y = leafsize3d;
        }
        else
        {
            x = leafsize3d;
            y = leafsize3d;
            z = leafsize3d*1.3;
        }
        sor.setInputCloud(cloud_data->makeShared());
        sor.setLeafSize(x, y, z);
        sor.filter(*cloudpoint);
        return cloudpoint;
    }
    // 統計濾波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_data, *cloud_copy);
        // printf("sizeF : %ld\n",cloud_data->size());
        sor2.setInputCloud(cloud_data->makeShared());
        sor2.setMeanK(MeanK);               // Set number of neighbors to be considered for averaging
        sor2.setNegative(false);
        sor2.setStddevMulThresh(MulThresh); // Set standard deviation multiplier


        return cloud_data;
    }

    // 地面演算法(sample consensus)
    void RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {

        pcl::SACSegmentation<pcl::PointXYZ> sac;
        sac.setInputCloud(cloud_data);
        sac.setMethodType(pcl::SAC_RANSAC);
        sac.setModelType(pcl::SACMODEL_PLANE);
        sac.setDistanceThreshold(0.05);
        sac.setMaxIterations(1000);
        // sac.setInputNormals(cloud_normals);
        sac.setProbability(0.95);
        sac.segment(*inliers, *coefficients);
    }

    // 移除地面
    void NoGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_data);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_data);
    }
    // 顯示地面
     void Ground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data){
         // Generate random color
         printf("hi");
         uint8_t r = 255;
         uint8_t g = 255;
         uint8_t b = 255;
         for (std::vector<int>::const_iterator pit = inliers->indices.begin(); pit != inliers->indices.end(); ++pit)
         {
             pcl::PointXYZRGB colored_point;
             colored_point.x = cloud_data->points[*pit].x;
             colored_point.y = cloud_data->points[*pit].y;
             colored_point.z = cloud_data->points[*pit].z;
             colored_point.r = r;
             colored_point.g = g;
             colored_point.b = b;
             outputcloudg->points.push_back(colored_point);
         }
         outputcloudg->width = outputcloudg->points.size();
         outputcloudg->height = 1;
         outputcloudg->is_dense = true;
     }
    //  映射降維度
    pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpoint;
        cloudpoint.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr mc(new pcl::ModelCoefficients);
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
        proj.filter(*cloudpoint);
        return cloudpoint;
    }

    // PCA
    // void PCA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    // {
    //     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    //     ne.setInputCloud(cloud_data);
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //     ne.setSearchMethod(tree);
    //     ne.setRadiusSearch(0.03);
    //     ne.compute(*cloud_normals);

    //     // pcl::Normal overall_normal(0, 0, 0);
    //     // // printf("0 = %f\n",overall_normal.normal_x);
    //     // for (size_t i = 0; i < cloud_normals->size(); ++i) {
    //     //     if(cloud_normals->points[i].normal_x!=cloud_normals->points[i].normal_x){
    //     //         // error++;
    //     //         // printf("time %d = %f\n",i,cloud_normals->points[i].normal_x);
    //     //         continue;
    //     //     }
    //     //     overall_normal.normal_x += cloud_normals->points[i].normal_x;
    //     //     overall_normal.normal_y += cloud_normals->points[i].normal_y;
    //     //     overall_normal.normal_z += cloud_normals->points[i].normal_z;

    //     //     // overall_normal.normal_x += abs(cloud_normals->points[i].normal_x);
    //     //     // overall_normal.normal_y += abs(cloud_normals->points[i].normal_y);
    //     //     // overall_normal.normal_z += abs(cloud_normals->points[i].normal_z);
    //     // }

    //     // if(normal_list.size()>=100){
    //     //     normal_list.erase(normal_list.begin());  // 刪除頭一筆資料
    //     // }
    //     // printf("time 1 = %f\n",overall_normal.normal_x);
    //     // normal_list.push_back(overall_normal);  // 新增資料到最後面
    // }

    // 半徑濾波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data, int MinNeighbors=0)
    {   if(MinNeighbors==0){
        MinNeighbors=MinNeighborsInRadius;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpoint;
        cloudpoint.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud_data);                     // 将点云输入到滤波器
        outrem.setRadiusSearch(RadiusSearch);                 // 设置近邻点搜索半径
        outrem.setMinNeighborsInRadius(MinNeighbors);           // 设置查询点最小近邻点数
        outrem.filter(*cloudpoint);
        return cloudpoint;

    }

    // DBSCANmain
    void DBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data, int param)
    {
        int DBminPts;
            DBminPts=minPts/param;


        clusterIndices.clear();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud;
        cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> clusterIndicesdata;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster3_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_data->makeShared());
        ec.setClusterTolerance(eps);
        ec.setMinClusterSize(DBminPts);
        // ec.setMaxClusterSize(max_points_per_cluster);
        ec.setMaxClusterSize(cloud_data->size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_data->makeShared());
        ec.extract(clusterIndicesdata);
        for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndicesdata.begin(); it != clusterIndicesdata.end(); ++it)
        {
            if (it->indices.size() > max_points_per_cluster)
            {
                // 將聚類中的點提取出來，用於二次聚類
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                {
                    cluster_cloud->points.push_back(cloud_data->points[*pit]);
                }
                cluster_cloud->width = cluster_cloud->points.size();
                cluster_cloud->height = 1;
                cluster_cloud->is_dense = true;

                // 使用RANSAC演算法進行二次聚類
                pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_LINE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(0.08);
                seg.setInputCloud(cluster_cloud);
                seg.segment(*inliers2, *coefficients2);

                // 將二次聚類的結果添加到聚類列表中
                std::vector<int> second_cluster_indices;
                for (std::vector<int>::const_iterator pit = inliers2->indices.begin(); pit != inliers2->indices.end(); ++pit)
                {
                    int index = it->indices[*pit];
                    second_cluster_indices.push_back(index);
                    
                }
                if (second_cluster_indices.size() > 0)
                {
                    pcl::PointIndices::Ptr second_indices(new pcl::PointIndices);
                    second_indices->indices = second_cluster_indices;
                    clusterIndices.push_back(*second_indices);

                    // 第三次聚類的結果添加到聚類列表中
                    pcl::ExtractIndices<pcl::PointXYZ> extract_remain;
                    extract_remain.setInputCloud(cluster_cloud);
                    extract_remain.setNegative(true);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr remain_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    extract_remain.filter(*remain_cloud);
                    ec.setInputCloud(remain_cloud);
                    std::vector<pcl::PointIndices> subClusterIndices;
                    ec.extract(subClusterIndices);
                    // printf("second_indices 2 = %ld\n",subClusterIndices.size());
                    if(subClusterIndices.size()>0){
                        for (std::vector<pcl::PointIndices>::const_iterator subit = subClusterIndices.begin(); subit != subClusterIndices.end(); ++subit)
                        {
                            clusterIndices.push_back(*subit);
                        }                              

                        
                    }
                }


            }
            else{
                clusterIndices.push_back(*it);
            }
        }

    }

    // void removeIndices( pcl::PointCloud<pcl::PointXYZ>::Ptr A,
    //                  pcl::PointIndices::Ptr B,
    //                 pcl::PointCloud<pcl::PointXYZ>::Ptr C) {
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(A);
    // extract.setIndices(B);
    // extract.setNegative(true);  // 仅提取A中不在B中的点云
    // extract.filter(*C);
    // }
    // 測試並顯示顏色
    void testcolor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        // printf("123");
        for(int i=0;i<cloud_data->size();i++){
            pcl::PointXYZRGB colored_point;
            colored_point.r = 255;
            colored_point.g = 255;
            colored_point.b = 255;
            colored_point.x = cloud_data->points[i].x;
            colored_point.y = cloud_data->points[i].y;
            colored_point.z = cloud_data->points[i].z;
            outputcloud->points.push_back(colored_point);      
        }

        // for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
        // {
        //     // if (it->indices.size() < 200){
        //     //     continue;
        //     // }
        //     pcl::PointXYZRGB colored_point;
        //     colored_point.r = rand() % 256;
        //     colored_point.g = rand() % 256;
        //     colored_point.b = rand() % 256;
        //     // ROS_INFO("passstartnum : %d", colored_point.r );
        //     //  Iterate through each point in the cluster
        //     for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        //     {
        //         // Set point color
        //         colored_point.x = cloud_data->points[*pit].x;
        //         colored_point.y = cloud_data->points[*pit].y;
        //         colored_point.z = cloud_data->points[*pit].z;

        //         // Add colored point to new cloud
        //         outputcloud->points.push_back(colored_point);
        //     }
        // }
        // Set point cloud header
        // ROS_INFO("Output cloud contains %ld points", clusterIndices.size());
        outputcloud->width = outputcloud->points.size();
        outputcloud->height = 1;
        outputcloud->is_dense = true;
    }

    void object_boxes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D, bool is_static)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud;
        cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.header.stamp = ros::Time::now();

        // marker.header.stamp = ros::Time();
        marker.ns = "basic_shapes";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 0;
        marker.scale.x = 0.01; // line width
        marker.lifetime = ros::Duration(0.5);
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;

        // Draw bounding boxes for each cluster
        int cluster_id = 0;

        // marker_array->markers.clear();

        for (auto indices : clusterIndices)
        {
            if(drawBoundingBox(marker, cloud3D, cloud2D,is_static, pcl::PointIndices::Ptr(new pcl::PointIndices(indices)), cluster_id)){
                cluster_id++;
                if(is_static){
                    marker_dynamic->markers.push_back(marker);   
                }
                else{
                    marker_static->markers.push_back(marker);  
                }
                marker.points.clear();                
            }
        }
        // smoothObjectBoxes({0,0,0,0},5,true);
        
        // printf("%d", cluster_id);

        // ROS_INFO("static size :%d\n", marker_static->markers.size());
        // ROS_INFO("dynamic size :%d\n", marker_dynamic->markers.size());
    }

    bool drawBoundingBox(visualization_msgs::Marker &marker, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3D, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D, bool is_static, pcl::PointIndices::Ptr indices, int id)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud;
        cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr rangecloud;
        rangecloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::copyPointCloud(*cloud2D, indices->indices, *cluster_cloud);
        marker.id = id;

        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cluster_cloud);
        feature_extractor.compute();
        pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;

        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        //判斷是否為動態物件
        if(!is_static){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            for(auto points:frame)
                if(isInsideRectangle(points,position_OBB.x,position_OBB.y)){
                    return false;
                }
        }

        // 投影OBB到XY平面
        Eigen::Vector3f x_axis, y_axis, z_axis;
        x_axis = rotational_matrix_OBB.col(0);
        y_axis = rotational_matrix_OBB.col(1);
        z_axis = rotational_matrix_OBB.col(2);
        x_axis[2] = 0.0;
        y_axis[2] = 0.0;
        Eigen::Matrix3f rotation_xy;
        rotation_xy << x_axis.normalized(), y_axis.normalized(), z_axis.normalized();
        Eigen::Vector3f center_xy(position_OBB.x, position_OBB.y, min_point_OBB.z);

        // 计算点云在Z轴上的AABB
        // pcl::PointXYZ min_point_Z, max_point_Z;
        // pcl::copyPointCloud(*cloud3D, indices->indices, *cluster_cloud);
        // pcl::getMinMax3D(*cluster_cloud, min_point_Z, max_point_Z);  // 使用轴索引为2的Z轴
        Eigen::Vector3f position_OBB_Eigen(position_OBB.x, position_OBB.y, position_OBB.z);
        pcl::CropBox<pcl::PointXYZ> crop_filter;
        crop_filter.setInputCloud(cloud3D);
        crop_filter.setMin(Eigen::Vector4f(min_point_OBB.x - 0.05, min_point_OBB.y - 0.05, -1, 1.0));
        crop_filter.setMax(Eigen::Vector4f(max_point_OBB.x + 0.05, max_point_OBB.y + 0.05, 3, 1.0));
        crop_filter.setRotation(rotational_matrix_OBB.col(2));
        crop_filter.setTranslation(position_OBB_Eigen);
        crop_filter.filter(*rangecloud);
        if(rangecloud->size()==0){
            return false;
        }
        pcl::PointXYZ min_point_box;

        pcl::PointXYZ max_point_box;

        pcl::getMinMax3D(*rangecloud, max_point_box, min_point_box);
        // printf("%ld", rangecloud->size());
        if (rangecloud->size() > 0)
        {
            if (min_point_box.z < -1)
            {
                min_point_box.z = rangecloud->points[0].z - 0.05;
            }
            if (max_point_box.z > 3)
            {
                max_point_box.z = rangecloud->points[0].z + 0.05;
            }
        }
        else
        {
            min_point_box.z = -1;
            max_point_box.z = 3;
        }

        //輸出匡  
        Eigen::Vector3f p1(min_point_OBB.x, min_point_OBB.y, min_point_box.z);
        Eigen::Vector3f p2(min_point_OBB.x, max_point_OBB.y, min_point_box.z);
        Eigen::Vector3f p3(max_point_OBB.x, max_point_OBB.y, min_point_box.z);
        Eigen::Vector3f p4(max_point_OBB.x, min_point_OBB.y, min_point_box.z);

        Eigen::Vector3f p5(min_point_OBB.x, min_point_OBB.y, max_point_box.z);
        Eigen::Vector3f p6(min_point_OBB.x, max_point_OBB.y, max_point_box.z);
        Eigen::Vector3f p7(max_point_OBB.x, max_point_OBB.y, max_point_box.z);
        Eigen::Vector3f p8(max_point_OBB.x, min_point_OBB.y, max_point_box.z);

        p1 = rotation_xy * p1 + center_xy;
        p2 = rotation_xy * p2 + center_xy;
        p3 = rotation_xy * p3 + center_xy;
        p4 = rotation_xy * p4 + center_xy;
        p5 = rotation_xy * p5 + center_xy;
        p6 = rotation_xy * p6 + center_xy;
        p7 = rotation_xy * p7 + center_xy;
        p8 = rotation_xy * p8 + center_xy;


        if(is_static){
            // std::array<double, 8> rect1 = {p1[0],p1[1],p2[0],p2[1],p3[0],p3[1],p4[0],p4[1]};
            frame.push_back({p1[0],p1[1],p2[0],p2[1],p3[0],p3[1],p4[0],p4[1]});
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }
        
        else{
            for(auto points:frame){
                if(checkOverlap(points,{p1[0],p1[1],p2[0],p2[1],p3[0],p3[1],p4[0],p4[1]})){
                    // printf("1 points = %lf %lf %lf %lf %lf %lf %lf %lf\n",points[0],points[1],points[2],points[3],points[4],points[5],points[6],points[7]);
                    // printf("2 points = %lf %lf %lf %lf %lf %lf %lf %lf\n",p1[0],p1[1],p2[0],p2[1],p3[0],p3[1],p4[0],p4[1]);
                    return false;
                }                
            }

        }

        // 将OBB的12条边添加到Marker中
        addLine(marker, p1, p2);
        addLine(marker, p2, p3);
        addLine(marker, p3, p4);
        addLine(marker, p4, p1);
        addLine(marker, p5, p6);
        addLine(marker, p6, p7);
        addLine(marker, p7, p8);
        addLine(marker, p8, p5);
        addLine(marker, p1, p5);
        addLine(marker, p2, p6);
        addLine(marker, p3, p7);
        addLine(marker, p4, p8);  

    
     
        return true; 
    }
    void addLine(visualization_msgs::Marker &marker, Eigen::Vector3f p1, Eigen::Vector3f p2)
    {
        geometry_msgs::Point p;
        p.x = p1(0);
        p.y = p1(1);
        p.z = p1(2);
        marker.points.push_back(p);
        p.x = p2(0);
        p.y = p2(1);
        p.z = p2(2);
        marker.points.push_back(p);
    }

    void TransformedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data)
    {
        Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f target_direction(0.0, 0.0, 1.0);
        Eigen::Quaternionf q;
        q.setFromTwoVectors(plane_normal, target_direction);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = q.toRotationMatrix();

        // 将点云转换到新方向
        // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_data, *cloud_data, transform);

        // 保存转换后的点云
        // pcl::io::savePLYFile ("transformed_cloud.ply", *transformed_cloud);
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr TemporalSmoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
    // {

    //     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //     icp.setInputCloud(cloud);
    //     icp.setInputTarget(cloud2);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;
    //     aligned_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    //     icp.align(*aligned_cloud);
    //     return aligned_cloud;
    // }
    // void Eu(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    //     //欧式分割器
    //     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //     ec.setClusterTolerance(0.14); // 2cm
    //     ec.setMinClusterSize(10);
    //     ec.setMaxClusterSize(500);
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        
    //     //搜索策略树
    //     ec.setSearchMethod(tree);
    //     ec.setInputCloud(cloud);
    //     ec.extract(clusterIndices);
    // }

    const pcl::PointCloud<pcl::PointXYZ>::Ptr AddPointCloudToBufferAndSum(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud, int BUFFER_SIZE) {
        // 將點雲加入循環緩衝區
        buffer.push_front(pointcloud);

        // 如果循環緩衝區超過設定的大小，則刪除最舊的點雲
        if (buffer.size() > BUFFER_SIZE) {
            buffer.pop_back();
        }

        // 清空加總後的點雲
        pcl::PointCloud<pcl::PointXYZ>::Ptr sumCloud;
        sumCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        // 將循環緩衝區中的點雲進行加總
        for (const auto& cloud1 : buffer) {
            std::copy(cloud1->begin(), cloud1->end(), std::back_inserter(*sumCloud));
        }
        return sumCloud;
    }

    bool isInsideRectangle( const std::array<double, 8>& rectangle, double px, double py)
    {
        
        double x1 = rectangle[0];
        double y1 = rectangle[1];
        double x2 = rectangle[2];
        double y2 = rectangle[3];
        double x3 = rectangle[4];
        double y3 = rectangle[5];
        double x4 = rectangle[6];
        double y4 = rectangle[7];

        // 检查点是否在矩形框的x范围内
        if (px < std::min(x1, std::min(x2, std::min(x3, x4))) || px > std::max(x1, std::max(x2, std::max(x3, x4))))
            return false;
            
        // 检查点是否在矩形框的y范围内
        if (py < std::min(y1, std::min(y2, std::min(y3, y4))) || py > std::max(y1, std::max(y2, std::max(y3, y4))))
            return false;

        return true;
    }
        
    // 檢查兩條線段是否相交
    bool doSegmentsIntersect(Point p1, Point p2, Point p3, Point p4) {
        double d1 = (p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x);
        double d2 = (p4.x - p3.x) * (p2.y - p3.y) - (p4.y - p3.y) * (p2.x - p3.x);
        double d3 = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        double d4 = (p2.x - p1.x) * (p4.y - p1.y) - (p2.y - p1.y) * (p4.x - p1.x);

        return (d1 * d2 < 0 && d3 * d4 < 0);
    }

    bool checkOverlap(const std::array<double, 8>& rect1, const std::array<double, 8>& rect2) {
        std::array<Point, 4> rect1Points = {{
            {rect1[0], rect1[1]},
            {rect1[2], rect1[3]},
            {rect1[4], rect1[5]},
            {rect1[6], rect1[7]}
        }};

        std::array<Point, 4> rect2Points = {{
            {rect2[0], rect2[1]},
            {rect2[2], rect2[3]},
            {rect2[4], rect2[5]},
            {rect2[6], rect2[7]}
        }};

        // 檢查所有線段是否相交
        for (int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            for (int k = 0; k < 4; k++) {
                int l = (k + 1) % 4;
                if (doSegmentsIntersect(rect1Points[i], rect1Points[j], rect2Points[k], rect2Points[l])) {
                    return true;  // 相交，有重疊
                }
            }
        }
        for (int i = 0; i < 2; i++) {
            int j = (i + 2) ;
            for (int k = 0; k < 2; k++) {
                int l = (k + 2) ;
                if (doSegmentsIntersect(rect1Points[i], rect1Points[j], rect2Points[k], rect2Points[l])) {
                    return true;  // 相交，有重疊
                }
            }
        }
        return false;  // 沒有重疊
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
void makerPublish(ros::Publisher marker_pub, visualization_msgs::MarkerArray::Ptr markerarray)
{
    // ROS_INFO("%c",markerarray->markers.size());
    visualization_msgs::Marker marker;
    marker.header = msg_header;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub.publish(marker);
    marker_pub.publish(*markerarray);
}
// void PCAPublish(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::list<pcl::Normal> normal_list)
// {
//     // 將法向量轉換為ROS Marker
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "velodyne";
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "point_cloud_normals";
//     marker.action = visualization_msgs::Marker::DELETEALL;
//     marker_pub2.publish(marker);
//     marker.type = visualization_msgs::Marker::ARROW;
//     marker.action = visualization_msgs::Marker::ADD;
    
//     marker.scale.x = 0.01;
//     marker.scale.y = 0.02;
//     marker.scale.z = 0.1;
//     marker.color.a = 1.0;
//     marker.color.r = 0.0;
//     marker.color.g = 1.0;
//     marker.color.b = 0.0;
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;

//     // printf("2 = %f\n",overall_normal.normal_x);
//     geometry_msgs::Point p, q;
//     p.x = q.x = 0;
//     p.y = q.y = 0;
//     p.z = q.z = 0;

//     // for(std::list<pcl::Normal>::iterator it = normal_list.begin(); it != normal_list.end(); ++it){
//     //     q.x += abs(it->normal_x);
//     //     q.y += abs(it->normal_y);
//     //     q.z += abs(it->normal_z);
//     // }
//     for (std::list<pcl::Normal>::iterator it = normal_list.begin(); it != normal_list.end(); ++it)
//     {
//         q.x = abs(it->normal_x);
//         q.y = abs(it->normal_y);
//         q.z = abs(it->normal_z);
//     }

//     marker.points.push_back(p);
//     marker.points.push_back(q);
//     marker_pub2.publish(marker);
// }

void pubplane(ros::Publisher marker_pub,pcl::ModelCoefficients::Ptr a)
{
    // 創建Marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "plane";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    if (a->values.size() < 3)
    {
        ROS_ERROR("計算的平面係數為空。");
        return;
    }
    double length = sqrt(a->values[0] * a->values[0] +
                         a->values[1] * a->values[1] +
                         a->values[2] * a->values[2]);
    
    // 將四元數設置到Marker的姿勢中
    marker.pose.position.x = 0;  // 平面係數的X分量
    marker.pose.position.y = 0;  // 平面係數的Y分量 
    marker.pose.position.z = 0;  // 平面係數的Z分量
    marker.pose.orientation.x =  a->values[0] / length;
    marker.pose.orientation.y =  a->values[1] / length;
    marker.pose.orientation.z =  a->values[2] / length;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 3.0;
    marker.scale.y = 3.0;
    marker.scale.z = 0.01;  // 平面的厚度
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();  // 持久存在，不自動刪除
    printf("123");
    // 發布Marker消息
    marker_pub.publish(marker);
}



int main(int argc, char **argv)
{
    // ROS 初始化
    ros::init(argc, argv, "pcl");
    ros::NodeHandle nh;
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, cloudCallback);
    // sub = nh.subsc   ribe<sensor_msgs::PointCloud2>("/yrl_pub/yrl_cloud", 1, cloudCallback);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/output_cloud", 1);
    marker_pub2 = nh.advertise<visualization_msgs::Marker>("point_cloud_normals", 1);
    ros::Publisher S_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("dynamic_object_boxes", 10000);
    ros::Publisher D_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("static_object_boxes", 10000);
    // ros::Publisher marker_pub3 = nh.advertise<visualization_msgs::Marker>("output", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker_topic", 1);
    dynamic_reconfigure::Server<dynamic_object_detection::pointcloudConfig> server;
    dynamic_reconfigure::Server<dynamic_object_detection::pointcloudConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point3D, point2D, pointcp , pointmap,pointmapmove;
    // std::vector<pcl::PointIndices> dbscan_data;
    ros::Rate loop_rate(ros_hz);
    clock_t start_time, end_time;
    double total_time;
    while (ros::ok())
    {
        ros::spinOnce();
        if (fisrt_data)
        {
            start_time = clock();
            point3D = PCM.start(input_cloud_data, msg_header);
            point3D = PCM.PassThroughFilter(point3D);
            point3D = PCM.Points_Simplification(point3D, false);
            // PCAPublish(PCM.cloud_normals,PCM.normal_list);
            point3D = PCM.StatisticalOutlier(point3D);
            PCM.RANSAC(point3D);
            PCM.NoGround(point3D);//!!!
            point2D = PCM.ProjectInliers(point3D);

            // printf("\nsize: %ld\n",point2D->size());
            // point2D = PCM.StatisticalOutlier(point2D);
            // PCM.testcolor(point2D);
            // // PCM.StatisticalOutlier(point2D);
            pointcp=PCM.RadiusOutlierRemoval(point2D,3);
            
            // PCM.dymap(pointmap,1);
            // printf("\nsize: %ld\n",point2D->size());
            // point2D = PCM.RadiusOutlierRemoval(point2D);;
            // printf("\nsize: %ld\n",point2D->size());
            pointmap =PCM.AddPointCloudToBufferAndSum(pointcp,20);
            pointmapmove=PCM.RadiusOutlierRemoval(pointmap,30);
            pointmapmove = PCM.Points_Simplification(pointmapmove, true);
            // if (pointcp==nullptr ){
            //     pointcp = point2D;
            // }

            // pointcp = PCM.TemporalSmoothing(point2D, pointcp);            
            // PCM.testcolor(point2D);
            // PCM.DBSCAN(point2D);
            PCM.DBSCAN(pointmapmove,3);
            PCM.object_boxes(point3D, pointmapmove,true);            
            point2D = PCM.RadiusOutlierRemoval(point2D);
            PCM.DBSCAN(point2D,1);
            PCM.object_boxes(point3D, point2D,false);
            

            // PCM.object_boxes(point3D, point2D,true);
            printf("  D size = %ld\n",PCM.marker_static->markers.size());
            makerPublish(S_marker_pub,PCM.marker_static);
            makerPublish(D_marker_pub,PCM.marker_dynamic);
            // PCM.Eu(point2D);
            // PCM.DBSCAN(point2D); 
            // PCM.object_boxes(point3D, point2D);
            // makerPublish(PCM.marker_array);

            // printf("\nsize: %ld\n",pointmap->size());
            PCM.testcolor(point2D);

            
            cloudPublish(PCM.outputcloud);
            end_time = clock();
            total_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
            cout << "The total time is: " << total_time << " seconds." << endl;

            pubplane(marker_pub,PCM.coefficients);

        }
        // ros::spinOnce();
        loop_rate.sleep();
    }
    // PCM.outimages();
    ros::spin();
    return 0;
}

// 更改參數
void callback(dynamic_object_detection::pointcloudConfig &config, uint32_t level)
{

    PCM.parameter(config);
    if (ros_hz != config.ros_hz)
    {
        ros::Rate loop_rate(config.ros_hz);
    }
    ros_hz = config.ros_hz;
}
