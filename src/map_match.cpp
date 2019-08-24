
/* 目標：srv使って
 * 今の位置と位置合わせの位置を与える
 * mapの位合わせをsrvで夜のを作る
 *
 */
#include"map_match.hpp"


Matcher::Matcher(ros::NodeHandle n,ros::NodeHandle private_nh_) :
	map_limit(20.0),
	lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	low_lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//今もらってきたレーザの点群
	map_cloud(new pcl::PointCloud<pcl::PointXYZI>),		//icp後の点群
	local_map_cloud(new pcl::PointCloud<pcl::PointXYZI>),//icp後の点群
	is_start(false)
{
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/vis/ndt", 10);
	map_pub = n.advertise<sensor_msgs::PointCloud2>("/vis/map", 1, true);
	odom_pub = n.advertise<nav_msgs::Odometry>("/sq_ndt_data", 10);

	pc_sub = n.subscribe("/velodyne_points", 10, &Matcher::lidarcallback, this);
	odom_sub = n.subscribe("/ekf_NDT", 10, &Matcher::odomcallback, this);
	
	ndt.setTransformationEpsilon(0.001);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);//1.0 change 05/09
	ndt.setMaximumIterations(35);

	private_nh_.getParam("voxel_size",voxel_size);
}


void
Matcher::map_read(string filename){

	pcl::PointCloud<pcl::PointXYZI>::Ptr low_map_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *low_map_cloud) == -1)  
		PCL_ERROR ("事前地図ないよ \n");
	else 
		cout<<"\x1b[31m"<<"読み込んだファイル："<<filename<<"\x1b[m\r"<<endl;

    pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
    
	approximate_voxel_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
	approximate_voxel_filter.setInputCloud (low_map_cloud);
	approximate_voxel_filter.filter (*map_cloud);


	sensor_msgs::PointCloud2 vis_map;
	pcl::toROSMsg(*map_cloud , vis_map);           

	vis_map.header.stamp = ros::Time::now(); //laserのframe_id
	// vis_map.header.stamp = ros::Time(0); //laserのframe_id
	vis_map.header.frame_id = "/map";

	map_pub.publish(vis_map);
	sleep(1.0);
	cout<<"\x1b[31m"<<"map read finish"<<filename<<"\x1b[m\r"<<endl;
}





void
Matcher::lidarcallback(const sensor_msgs::PointCloud2::Ptr msg){

	low_lidar_cloud->points.clear();

	pcl::fromROSMsg(*msg,*lidar_cloud);

	for(pcl::PointXYZI temp_point : lidar_cloud->points){
		
		if((map_limit * (-1) <= temp_point.x && temp_point.x  <= map_limit) && (map_limit *(-1) <= temp_point.y && temp_point.y <= map_limit) ){
	 
			low_lidar_cloud->points.push_back(temp_point);
		}
	}
	//ここでvoxel化かけると重くなる(処理に関係ない点群をvoxel化するため)
}

void
Matcher::odomcallback(const nav_msgs::OdometryConstPtr& msg){
	is_start = true;
	buffer_odom = *msg;

}



//位置合わせ
void
Matcher::aligner_ndt(geometry_msgs::Pose& now_pose){

	double roll,pitch,yaw;
	tf::Quaternion q;
	// quaternionMsgToTF(now_pose.orientation,q);
	// tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
	yaw = now_pose.orientation.z;
 
	Eigen::AngleAxisf rot (yaw, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation (now_pose.position.x, now_pose.position.y, -1.3);
    
	Eigen::Matrix4f transform = (init_translation * rot).matrix (); //行列の掛け算には注意
    
	
	local_map(map_cloud,now_pose.position.x, now_pose.position.y);
	
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
	pcl::PointCloud<pcl::PointXYZI>::Ptr limit_lidar_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    
	approximate_voxel_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
	approximate_voxel_filter.setInputCloud (low_lidar_cloud);
	approximate_voxel_filter.filter (*limit_lidar_cloud);
	
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr answer_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	
	
	ndt.setInputTarget(local_map_cloud);	//map
	ndt.setInputSource(limit_lidar_cloud);	//lidar
	ndt.align (*answer_cloud, transform);			//移動後のlidar
	// ndt.align (*answer_cloud);			//faster
	pc_publisher(answer_cloud,"/map");

 ////answer
 	Eigen::Matrix4f a;
 	a = ndt.getFinalTransformation ();

 	tf::Matrix3x3 mat_l;
	double l_roll, l_pitch, l_yaw;//角度etc
 
	mat_l.setValue(static_cast<double>(a(0, 0)), static_cast<double>(a(0, 1)), static_cast<double>(a(0, 2)),
 			static_cast<double>(a(1, 0)), static_cast<double>(a(1, 1)), static_cast<double>(a(1, 2)),
 			static_cast<double>(a(2, 0)), static_cast<double>(a(2, 1)), static_cast<double>(a(2, 2)));

 	mat_l.getRPY(l_roll, l_pitch, l_yaw, 1);

	// cout << "Result : " << endl;
	// cout << ndt.getFinalTransformation()<<endl;

	tf::Quaternion quat = tf::createQuaternionFromRPY(l_roll,l_pitch,l_yaw);
	
	geometry_msgs::Quaternion geometry_quat;
	quaternionTFToMsg(quat, geometry_quat);

	now_pose.position.x = a(0, 3);
	now_pose.position.y = a(1, 3);
	now_pose.position.z = 0.0;
	// now_pose.orientation = geometry_quat;
	now_pose.orientation.z = l_yaw;
}


//mapの一部を算出
void 
Matcher::local_map(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,double x_now,double y_now)
{
    //limit_laserを設定
	local_map_cloud->points.clear();

	for(pcl::PointXYZI temp_point :input_cloud->points){
		
		if((map_limit * (-1) + x_now <= temp_point.x && temp_point.x  <= map_limit + x_now) && (map_limit *(-1) + y_now <= temp_point.y && temp_point.y <= map_limit + y_now) ){
	 
			local_map_cloud->points.push_back(temp_point);
		}
	}
}



void
Matcher::pc_publisher(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,string frame_id){

	sensor_msgs::PointCloud2 vis_pc;
	pcl::toROSMsg(*cloud , vis_pc);           
	
	vis_pc.header.stamp = ros::Time::now();
	vis_pc.header.frame_id = frame_id;

	pc_pub.publish(vis_pc);
}


void
Matcher::process(){
	
	aligner_ndt(buffer_odom.pose.pose);
	
	buffer_odom.header.frame_id = "/map";
	buffer_odom.child_frame_id = "/matching_base_link";

	odom_pub.publish(buffer_odom);
}

