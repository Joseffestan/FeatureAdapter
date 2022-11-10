#pragma once
#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <time.h>
#include <assert.h>

using namespace std;
using namespace pcl;
using namespace Eigen;


//一点的PFH数组
struct PFH
{
	float fHistogram[125];
	float x;
	float y;
	float z;
	float ref;
	void* feature_data;//备份PFH特征信息

};
//一点的FPFH数组
struct FPFH
{
	float fHistogram[33];
	float x;
	float y;
	float z;
};
extern Vector3d GetPointNormal(PointCloud<PointXYZRGB>::Ptr Cloud, int Idx, int NeiborDistance, vector<int> &NeiborPointsIdx);

extern Vector3d GetNeiborNormal(PointCloud<PointXYZRGB>::Ptr Cloud, int Idx, int NeiborDistance);

extern PointCloud<Normal>::Ptr GetCloudNormal(PointCloud<PointXYZRGB>::Ptr Cloud, int iNeiborDistance = 400);

extern void visualize_pcd(PointCloud<PointXYZ>::Ptr pcd1, PointCloud<PointXYZ>::Ptr pcd2, PointCloud<PointXYZ>::Ptr pcd3, PointCloud<PointXYZ>::Ptr pcd4);

extern PointCloud<PointXYZRGB>::Ptr CloudFeatureExtraction(PointCloud<PointXYZRGB>::Ptr InputCloud, PointCloud<Normal>::Ptr FeatureNormals, int iNeiborsDistance = 400,int iDiffPointNumThrsd = 5);

extern PFH* CalculatePFH(PointCloud<PointXYZRGB>::Ptr Cloud, PointCloud<Normal>::Ptr Normal, int iSearchRadius = 400);

extern vector<FPFH> CalculateFPFH(PointCloud<PointXYZRGB>::Ptr Cloud, PointCloud<Normal>::Ptr Normal, int iSearchRadius = 400);

extern PFH* LoadPFHFile(char* strPFHFileName, int *num, int* id);

extern void ShowProgress(int flag, int fullLenth);
