#include "stdafx.h"
#include "pfh.h"

//计算两点距离的平方
float PointDistance(PointXYZRGB P1, PointXYZRGB P2)
{
	float ans = ((P1.x - P2.x)*(P1.x - P2.x) + (P1.y - P2.y)*(P1.y - P2.y) + (P1.z - P2.z)*(P1.z - P2.z));
	return ans;
}
//计算某点法向量
Vector3d GetPointNormal(PointCloud<PointXYZRGB>::Ptr Cloud, int Idx, int NeiborDistance, vector<int> &NeiborPointsIdx)
{
	Vector3d vCurPoint = { Cloud->at(Idx).x,Cloud->at(Idx).y, Cloud->at(Idx).z };
	for (int i = 0; i < Cloud->width; ++i)//邻域点查找
	{
		if (PointDistance(Cloud->at(Idx), Cloud->at(i)) < NeiborDistance)//邻域范围
			NeiborPointsIdx.push_back(i);
	}
	PointXYZ pNeiborHeart;
	//计算邻域点质心
	float fSumX(0.0), fSumY(0.0), fSumZ(0.0);
	for (int i = 0; i < NeiborPointsIdx.size(); ++i)
	{
		pNeiborHeart.x += 1;
		fSumX += Cloud->at(NeiborPointsIdx.at(i)).x;
		fSumY += Cloud->at(NeiborPointsIdx.at(i)).y;
		fSumZ += Cloud->at(NeiborPointsIdx.at(i)).z;
	}
	pNeiborHeart.x = fSumX / NeiborPointsIdx.size();
	pNeiborHeart.y = fSumY / NeiborPointsIdx.size();
	pNeiborHeart.z = fSumZ / NeiborPointsIdx.size();
	//计算协方差矩阵
	Matrix3d Conv = Matrix3d::Zero();
	Vector3d vNeiborHeart = { pNeiborHeart.x, pNeiborHeart.y, pNeiborHeart.z };
	Vector3d vNeiborPoint;
	for (int i = 0; i < NeiborPointsIdx.size(); ++i)
	{
		vNeiborPoint = { Cloud->at(NeiborPointsIdx.at(i)).x,Cloud->at(NeiborPointsIdx.at(i)).y, Cloud->at(NeiborPointsIdx.at(i)).z };
		Conv += (vNeiborPoint - vNeiborHeart)*(vNeiborPoint - vNeiborHeart).transpose();
	}
	Conv /= NeiborPointsIdx.size();
	//找到协方差矩阵的最小特征值
	SelfAdjointEigenSolver<Matrix3d> EigenSolver(Conv);
	Vector3d vEigenValues = EigenSolver.eigenvalues();//特征值
	Matrix3d mEigenVectors = EigenSolver.eigenvectors();//特征向量
	MatrixXf::Index MinEigenValueIdx;//最小特征值位置
	vEigenValues.real().rowwise().sum().minCoeff(&MinEigenValueIdx);
	Vector3d vMinEigenVector;
	vMinEigenVector << mEigenVectors.real()(0, MinEigenValueIdx), mEigenVectors.real()(1, MinEigenValueIdx), mEigenVectors.real()(2, MinEigenValueIdx);
	if (vMinEigenVector.dot(-vCurPoint) < 0)//法向量和坐标向量的点积
		vMinEigenVector = -vMinEigenVector;
	return vMinEigenVector;
}
//计算邻域点法向量
Vector3d GetNeiborNormal(PointCloud<PointXYZRGB>::Ptr Cloud, int Idx, int NeiborDistance)
{
	vector<int> vNeiborPointsIdx;//邻域点索引
	Vector3d vCurPoint = { Cloud->at(Idx).x,Cloud->at(Idx).y, Cloud->at(Idx).z };
	for (int i = 0; i < Cloud->width; ++i)//邻域点查找
	{
		if (PointDistance(Cloud->at(Idx), Cloud->at(i)) < NeiborDistance)//邻域范围
			vNeiborPointsIdx.push_back(i);
	}
	PointXYZ pNeiborsHeart;
	//计算邻域点质心
	float fSumX(0.0), fSumY(0.0), fSumZ(0.0);
	for (int i = 0; i < vNeiborPointsIdx.size(); ++i)
	{
		fSumX += Cloud->at(vNeiborPointsIdx.at(i)).x;
		fSumY += Cloud->at(vNeiborPointsIdx.at(i)).y;
		fSumZ += Cloud->at(vNeiborPointsIdx.at(i)).z;
	}
	pNeiborsHeart.x = fSumX / vNeiborPointsIdx.size();
	pNeiborsHeart.y = fSumY / vNeiborPointsIdx.size();
	pNeiborsHeart.z = fSumZ / vNeiborPointsIdx.size();
	//计算协方差矩阵
	Matrix3d Conv = Matrix3d::Zero();
	Vector3d vNeiborsHeart = { pNeiborsHeart.x, pNeiborsHeart.y, pNeiborsHeart.z };
	Vector3d vNeiborsPoint;
	for (int i = 0; i < vNeiborPointsIdx.size(); ++i)
	{
		vNeiborsPoint = { Cloud->at(vNeiborPointsIdx.at(i)).x,Cloud->at(vNeiborPointsIdx.at(i)).y, Cloud->at(vNeiborPointsIdx.at(i)).z };
		Conv += (vNeiborsPoint - vNeiborsHeart)*(vNeiborsPoint - vNeiborsHeart).transpose();
	}
	Conv /= vNeiborPointsIdx.size();
	//找到协方差矩阵的最小特征值
	SelfAdjointEigenSolver<Matrix3d> EigenSolver(Conv);
	Vector3d vEigenValues = EigenSolver.eigenvalues();//特征值
	Matrix3d mEigenVectors = EigenSolver.eigenvectors();//特征向量
	MatrixXf::Index MinEigenValueIdx;//最小特征值位置
	vEigenValues.real().rowwise().sum().minCoeff(&MinEigenValueIdx);
	Vector3d vMinEigenVector;
	vMinEigenVector << mEigenVectors.real()(0, MinEigenValueIdx), mEigenVectors.real()(1, MinEigenValueIdx), mEigenVectors.real()(2, MinEigenValueIdx);
	if (vMinEigenVector.dot(-vCurPoint) < 0)//法向量和坐标向量的点积
		vMinEigenVector = -vMinEigenVector;
	return vMinEigenVector;
}
//计算点云内所有点的法向量
PointCloud<Normal>::Ptr GetCloudNormal(PointCloud<PointXYZRGB>::Ptr Cloud, int iNeiborDistance)
{
	PointCloud<Normal>::Ptr Normals(new PointCloud<Normal>);
	for (int iter = 0; iter < Cloud->size(); ++iter)
	{
		vector<int> nNeiborPointsIdx;
		Vector3d vNormal = GetPointNormal(Cloud, iter, iNeiborDistance, nNeiborPointsIdx);//获得法向量
		cout << iter << "-" << nNeiborPointsIdx.size() << " ";
		Normal nNormal(vNormal(0, 0), vNormal(1, 0), vNormal(2, 0));
		Normals->push_back(nNormal);
	}
	return Normals;
}
//点云可视化
void visualize_pcd(PointCloud<PointXYZ>::Ptr pcd1, PointCloud<PointXYZ>::Ptr pcd2, PointCloud<PointXYZ>::Ptr pcd3, PointCloud<PointXYZ>::Ptr pcd4)
{
	//int vp_1, vp_2;
	// 创建一个PCLVisualizer对象
	pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");
	//viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	// viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler1(pcd1, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler2(pcd2, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler3(pcd3, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler4(pcd4, 0, 0, 255);
	viewer.addPointCloud(pcd1, handler1, "cloud one");
	viewer.addPointCloud(pcd2, handler2, "cloud two");
	viewer.addPointCloud(pcd3, handler3, "cloud three");
	viewer.addPointCloud(pcd4, handler4, "cloud four");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud three");//修改点云显示大小
	//viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
//点云特征提取
PointCloud<PointXYZRGB>::Ptr CloudFeatureExtraction(PointCloud<PointXYZRGB>::Ptr InputCloud, PointCloud<Normal>::Ptr FeatureNormals, int iNeiborsDistance ,int iDiffPointsNumThrsd)
{
	PointCloud<PointXYZRGB>::Ptr CloudVoxeled(new PointCloud<PointXYZRGB>);//体素滤波后点云
	PointCloud<PointXYZRGB>::Ptr CloudFiltered(new PointCloud<PointXYZRGB>);//离群点剔除后点云
	//体素滤波
	VoxelGrid<PointXYZRGB> VoxelFilter;
	VoxelFilter.setInputCloud(InputCloud);
	VoxelFilter.setLeafSize(10, 10, 10);//体素大小
	VoxelFilter.filter(*CloudVoxeled);
	//离群点滤波
	RadiusOutlierRemoval<PointXYZRGB> OutlierFilter;
	OutlierFilter.setInputCloud(CloudVoxeled);
	OutlierFilter.setRadiusSearch(40);// 搜索半径
	OutlierFilter.setMinNeighborsInRadius(5);//最少的邻居数目
	OutlierFilter.filter(*CloudFiltered);
	//为CloudFiltered的rgb赋值
	for (int i = 0; i < CloudFiltered->size(); ++i)
	{
		for (int j = 0; j < InputCloud->size(); ++j)
		{
			if ((fabsf(CloudFiltered->at(i).x - InputCloud->at(j).x) < 10) && (fabsf(CloudFiltered->at(i).y - InputCloud->at(j).y) < 10) && (fabsf(CloudFiltered->at(i).z - InputCloud->at(j).z) < 10))
			{ 
				CloudFiltered->at(i).rgb = InputCloud->at(j).rgb;
				break;
			}
		}
	}
	//遍历寻找特征点
	PointCloud<PointXYZRGB>::Ptr FeatureCloud(new PointCloud<PointXYZRGB>);
	for (int iter = 0; iter < CloudFiltered->size(); ++iter)
	{
		ShowProgress(iter, CloudFiltered->size());//显示进度
		vector<int> nNeiborPointsIdx;
		Vector3d vNormal = GetPointNormal(CloudFiltered, iter, iNeiborsDistance, nNeiborPointsIdx);//获得法向量
		vector<Vector3d> vNeiborEigenNormals;
		for (int i = 0; i < nNeiborPointsIdx.size(); ++i)//计算邻域点法向量
			vNeiborEigenNormals.push_back(GetNeiborNormal(CloudFiltered, nNeiborPointsIdx.at(i), iNeiborsDistance));
		float dThreshold = 0.85;//阈值 判断是否为特征点(0.8-0.9)
		int iDiffPointsNum = 0;//邻域差异点数量
		for (int i = 0; i < nNeiborPointsIdx.size(); ++i)
		{
			if (abs(vNormal.dot(vNeiborEigenNormals.at(i))) < dThreshold)
				iDiffPointsNum++;
		}
		if (iDiffPointsNum > iDiffPointsNumThrsd)//超过阈值的邻域点数量(1、2、3、4)
		{
			FeatureCloud->push_back(CloudFiltered->at(iter));
			Normal nNormal(vNormal(0, 0), vNormal(1, 0), vNormal(2, 0));
			FeatureNormals->push_back(nNormal);
		}
	}
	return FeatureCloud;
}

//计算点对四元组
bool computePointPairFeatures(const Vector4f &p1, const Vector4f &n1, const Vector4f &p2, const Vector4f &n2, float &f1, float &f2, float &f3, float &f4)
{
	Vector4f vP2_P1 = p2 - p1;
	vP2_P1[3] = 0.0f;
	f4 = vP2_P1.norm();// 计算点对的距离f4
	if (f4 == 0.0f)//检查f4错误
		return false;
	//计算f3，判定source和target
	Vector4f vSource = n1, vTarget = n2;//默认p1为source p2为target
	vSource[3] = vTarget[3] = 0.0f;
	float fAngle1 = vSource.dot(vP2_P1) / f4;
	float fAngle2 = vTarget.dot(vP2_P1) / f4;
	if (acos(fabs(fAngle1)) > acos(fabs(fAngle2)))//n2和p1p2夹角较小
	{//p2为source  p1为target
		vSource = n2;
		vTarget = n1;
		vSource[3] = vTarget[3] = 0.0f;
		vP2_P1 *= -1.0;
		f3 = -fAngle2;
	}
	else
		f3 = fAngle1;
	Vector4f v = vP2_P1.cross3(vSource);//Darboux坐标系v坐标轴
	v[3] = 0.0f;
	float fNormV = v.norm();
	if (fNormV == 0.0f)	//检查结果模长
		return false;
	v /= fNormV;// 标准化v
	Vector4f w = vSource.cross3(v);//Darboux坐标系w坐标轴
	v[3] = 0.0f;
	f2 = v.dot(vTarget);
	w[3] = 0.0f;
	f1 = atan2(w.dot(vTarget), vSource.dot(vTarget));
	return true;
}
//计算点云PFH,返回PFH数组的指针
PFH* CalculatePFH(PointCloud<PointXYZRGB>::Ptr Cloud, PointCloud<Normal>::Ptr Normal, int iSearchRadius)
{
	//vector<PFH> FinalPFH;
	PFH* FinalPFH = new PFH[Cloud->size()];
	PFH* PFHIter = FinalPFH;
	float fPFHTuples[4] = { 0.0,0.0,0.0,0.0 };//点对PFH四元组
	int iFeatureBinIndex[3];//点对四要素标准化的索引
	int iBinNum = 5;//特征值范围的子区间bin个数
	int iHistogramIndex = 0;//特征值归入直方图的索引
	int iMulti = 1;//特征值归入直方图的计算乘数
	for (int iter = 0; iter < Cloud->size(); ++iter)
	{
		PFH PointPFH;
		PointPFH.x = Cloud->at(iter).x;
		PointPFH.y = Cloud->at(iter).y;
		PointPFH.z = Cloud->at(iter).z;
		PointPFH.ref = Cloud->at(iter).rgb;
		for (int i = 0; i < 125; ++i)
			PointPFH.fHistogram[i] = 0.0;
		//查找邻域点
		vector<int> vNeiborPointIdx;
		for (int i = 0; i < Cloud->size(); ++i)
		{
			if (PointDistance(Cloud->at(iter), Cloud->at(i)) < iSearchRadius)
				vNeiborPointIdx.push_back(i);
		}
		float fHistogramIncrease = 100.0f / (vNeiborPointIdx.size() * (vNeiborPointIdx.size() - 1) / 2);//直方图单位递增量
		//计算该点的PFH
		for (int i = 0; i < vNeiborPointIdx.size(); ++i)
		{
			for (int j = 0; j < i; ++j)
			{
				Vector4f p1(Cloud->at(vNeiborPointIdx.at(i)).x, Cloud->at(vNeiborPointIdx.at(i)).y, Cloud->at(vNeiborPointIdx.at(i)).z, 1.0f);
				Vector4f p2(Cloud->at(vNeiborPointIdx.at(j)).x, Cloud->at(vNeiborPointIdx.at(j)).y, Cloud->at(vNeiborPointIdx.at(j)).z, 1.0f);
				Vector4f n1(Normal->at(vNeiborPointIdx.at(i)).normal_x, Normal->at(vNeiborPointIdx.at(i)).normal_y, Normal->at(vNeiborPointIdx.at(i)).normal_z, 0.0f);
				Vector4f n2(Normal->at(vNeiborPointIdx.at(j)).normal_x, Normal->at(vNeiborPointIdx.at(j)).normal_y, Normal->at(vNeiborPointIdx.at(j)).normal_z, 0.0f);
				computePointPairFeatures(p1, n1, p2, n2, fPFHTuples[0], fPFHTuples[1], fPFHTuples[2], fPFHTuples[3]);
				//标准化点对四要素到0 1 2 3 4
				iFeatureBinIndex[0] = floor(iBinNum * ((fPFHTuples[0] + M_PI) * (1.0 / (2.0*M_PI))));
				if (iFeatureBinIndex[0] < 0)        iFeatureBinIndex[0] = 0;
				if (iFeatureBinIndex[0] >= iBinNum) iFeatureBinIndex[0] = iBinNum - 1;
				iFeatureBinIndex[1] = floor(iBinNum * ((fPFHTuples[1] + 1.0) * 0.5));
				if (iFeatureBinIndex[1] < 0)        iFeatureBinIndex[1] = 0;
				if (iFeatureBinIndex[1] >= iBinNum) iFeatureBinIndex[1] = iBinNum - 1;
				iFeatureBinIndex[2] = floor(iBinNum * ((fPFHTuples[2] + 1.0) * 0.5));
				if (iFeatureBinIndex[2] < 0)        iFeatureBinIndex[2] = 0;
				if (iFeatureBinIndex[2] >= iBinNum) iFeatureBinIndex[2] = iBinNum - 1;
				// 归入直方图
				iHistogramIndex = 0;
				iMulti = 1;
				for (int d = 0; d < 3; ++d)
				{
					iHistogramIndex += iMulti * iFeatureBinIndex[d];
					iMulti *= iBinNum;
				}
				PointPFH.fHistogram[iHistogramIndex] += fHistogramIncrease;
			}
		}
		//FinalPFH.push_back(PointPFH);
		PFHIter->x = PointPFH.x;
		PFHIter->y = PointPFH.y;
		PFHIter->z = PointPFH.z;
		PFHIter->ref = PointPFH.ref;
		for (int ii = 0; ii < 125; ++ii)
			PFHIter->fHistogram[ii] = PointPFH.fHistogram[ii];
		PFHIter++;
	}
	return FinalPFH;
}
//计算点云FPFH
vector<FPFH> CalculateFPFH(PointCloud<PointXYZRGB>::Ptr Cloud, PointCloud<Normal>::Ptr Normal, int iSearchRadius)
{ 
	//计算SPFH到fSPFHHistF123中
	vector<FPFH> FinalFPFH;
	MatrixXf fSPFHHistF1, fSPFHHistF2, fSPFHHistF3;
	int iBinsNumF1 = 11, iBinsNumF2 = 11, iBinsNumF3 = 11;//直方图分区间数 
	fSPFHHistF1.setZero(Cloud->size(), iBinsNumF1);
	fSPFHHistF2.setZero(Cloud->size(), iBinsNumF2);
	fSPFHHistF3.setZero(Cloud->size(), iBinsNumF3);
	vector<int> vNearestPointsIdx;
	vector<float> vNearestPointsDist;
	float fPointsDist = 0.0;
	Vector4f fFPFHTuples; for (int iter = 0; iter < Cloud->size(); ++iter)
	{
		vNearestPointsIdx.clear();
		for (int i = 0; i < Cloud->size(); ++i)
		{//寻找近邻点
			fPointsDist = PointDistance(Cloud->at(iter), Cloud->at(i));
			if (fPointsDist < iSearchRadius)
				vNearestPointsIdx.push_back(i);
		}
		float fHistogramIncrease = 100.0f / (vNearestPointsIdx.size() - 1);
		for (size_t idx = 0; idx < vNearestPointsIdx.size(); ++idx)
		{
			if (0 == PointDistance(Cloud->at(iter), Cloud->at(vNearestPointsIdx.at(idx))))
				continue;//避免和自己进行计算
			//计算点对四元组
			Vector4f p1(Cloud->at(iter).x, Cloud->at(iter).y, Cloud->at(iter).z, 1.0f);
			Vector4f p2(Cloud->at(vNearestPointsIdx.at(idx)).x, Cloud->at(vNearestPointsIdx.at(idx)).y, Cloud->at(vNearestPointsIdx.at(idx)).z, 1.0f);
			Vector4f n1(Normal->at(iter).normal_x, Normal->at(iter).normal_y, Normal->at(iter).normal_z, 0.0f);
			Vector4f n2(Normal->at(vNearestPointsIdx.at(idx)).normal_x, Normal->at(vNearestPointsIdx.at(idx)).normal_y, Normal->at(vNearestPointsIdx.at(idx)).normal_z, 0.0f);
			computePointPairFeatures(p1, n1, p2, n2, fFPFHTuples[0], fFPFHTuples[1], fFPFHTuples[2], fFPFHTuples[3]);
			//标准化f1, f2, f3并写入直方图N
			int iHistBinIndex = floor(iBinsNumF1 * ((fFPFHTuples[0] + M_PI) * (1.0 / (2.0*M_PI))));
			if (iHistBinIndex < 0) iHistBinIndex = 0;
			if (iHistBinIndex >= iBinsNumF1) iHistBinIndex = iBinsNumF1 - 1;
			fSPFHHistF1(iter, iHistBinIndex) += fHistogramIncrease;
			iHistBinIndex = floor(iBinsNumF2 * ((fFPFHTuples[1] + 1.0) * 0.5));
			if (iHistBinIndex < 0) iHistBinIndex = 0;
			if (iHistBinIndex >= iBinsNumF2) iHistBinIndex = iBinsNumF2 - 1;
			fSPFHHistF2(iter, iHistBinIndex) += fHistogramIncrease;
			iHistBinIndex = floor(iBinsNumF3 * ((fFPFHTuples[2] + 1.0) * 0.5));
			if (iHistBinIndex < 0) iHistBinIndex = 0;
			if (iHistBinIndex >= iBinsNumF3) iHistBinIndex = iBinsNumF3 - 1;
			fSPFHHistF3(iter, iHistBinIndex) += fHistogramIncrease;
		}
	}
	//对SPFH加权得到FPFH
	FPFH PointFPFH;//某点的SPFH
	for (int iter = 0; iter < Cloud->size(); ++iter)
	{
		PointFPFH.x = Cloud->at(iter).x;
		PointFPFH.y = Cloud->at(iter).y;
		PointFPFH.z = Cloud->at(iter).z;
		vNearestPointsIdx.clear();
		vNearestPointsDist.clear();
		for (int i = 0; i < 33; ++i)
			PointFPFH.fHistogram[i] = 0.0;
		for (int i = 0; i < Cloud->size(); ++i)
		{//寻找近邻点
			fPointsDist = PointDistance(Cloud->at(iter), Cloud->at(i));
			if (fPointsDist < iSearchRadius)
			{
				vNearestPointsIdx.push_back(i);
				vNearestPointsDist.push_back(fPointsDist);
			}
		}
		float fSumF1 = 0.0, fSumF2 = 0.0, fSumF3 = 0.0;//f1 f2 f3各自值求和 准备标准化
		float fWeight = 0.0;//SPFH加权权重
		float fValueF1, fValueF2, fValueF3;//加权后的f1 f2 f3值
		// 加权该点的所有邻域点的SPFH到PointFPFH
		for (int idx = 0; idx < vNearestPointsIdx.size(); ++idx)
		{
			if (vNearestPointsDist[idx] == 0.0) continue;
			fWeight = 1.0f / vNearestPointsDist[idx];//该邻域点的权重
			for (int f1_i = 0; f1_i < iBinsNumF1; ++f1_i)//加权计算FPFH
			{
				fValueF1 = fSPFHHistF1(vNearestPointsIdx.at(idx), f1_i) * fWeight;
				fSumF1 += fValueF1;
				PointFPFH.fHistogram[f1_i] += fValueF1;
			}
			for (int f2_i = 0; f2_i < iBinsNumF2; ++f2_i)
			{
				fValueF2 = fSPFHHistF2(vNearestPointsIdx.at(idx), f2_i) * fWeight;
				fSumF2 += fValueF2;
				PointFPFH.fHistogram[f2_i + iBinsNumF1] += fValueF2;
			}
			for (int f3_i = 0; f3_i < iBinsNumF3; ++f3_i)
			{
				fValueF3 = fSPFHHistF3(vNearestPointsIdx.at(idx), f3_i) * fWeight;
				fSumF3 += fValueF3;
				PointFPFH.fHistogram[f3_i + iBinsNumF1 + iBinsNumF2] += fValueF3;
			}
		}
		if (fSumF1 != 0) fSumF1 = 100.0f / fSumF1;//点的直方图求和标准化为100
		if (fSumF2 != 0) fSumF2 = 100.0f / fSumF2;
		if (fSumF3 != 0) fSumF3 = 100.0f / fSumF3;
		for (int f1_i = 0; f1_i < iBinsNumF1; ++f1_i)
			PointFPFH.fHistogram[f1_i] *= fSumF1;
		for (int f2_i = 0; f2_i < iBinsNumF2; ++f2_i)
			PointFPFH.fHistogram[f2_i + iBinsNumF1] *= fSumF2;
		for (int f3_i = 0; f3_i < iBinsNumF3; ++f3_i)
			PointFPFH.fHistogram[f3_i + iBinsNumF1 + iBinsNumF2] *= fSumF3;
		FinalFPFH.push_back(PointFPFH);
	}
	return FinalFPFH;
}
//读取PFH文件
PFH* LoadPFHFile(char* strPFHFileName, int *num, int* id)
{
	//vector<PFH> FinalPFH;
	FILE* pfInputPFH;
	fopen_s(&pfInputPFH, strPFHFileName, "r");
	if (pfInputPFH == NULL) return NULL;
	fscanf(pfInputPFH, "%d %d\n", num, id);
	PFH* FinalPFH = new PFH[*num];
	PFH PointPFH;
	PointXYZRGB point;
	for (int i = 0; i < *num; ++i)
	{
		//fscanf(pfInputPFH, "(%f,%f,%f):", &FinalPFH[i].x, &FinalPFH[i].y, &FinalPFH[i].z);
		fscanf(pfInputPFH, "(%f,%f,%f,%f):", &point.x, &point.y, &point.z, &point.rgb);
		FinalPFH[i].x = point.x * 1000;
		FinalPFH[i].y = point.y * 1000;
		FinalPFH[i].z = point.z * 1000;
		FinalPFH[i].ref = point.rgb;
		for (int j = 0; j < 125; ++j)
			fscanf(pfInputPFH, "%f ", &FinalPFH[i].fHistogram[j]);
		fscanf(pfInputPFH, "\n");
		//FinalPFH.push_back(PointPFH);
	}
	fclose(pfInputPFH);
	return FinalPFH;
}

void ShowProgress(int flag, int fullLenth)
{
	if (fullLenth == 0)
		return;
	int step = fullLenth / 40;
	
	if(flag==0) cout << "0%";
	else if (flag == step)cout << ".";
	else if (flag == step * 2)cout << ".";
	else if (flag == step * 3)cout << ".";
	else if (flag == step * 4)cout << "10%";
	else if (flag == step * 5)cout << ".";
	else if (flag == step * 6)cout << ".";
	else if (flag == step * 7)cout << ".";
	else if (flag == step * 8)cout << "20%";
	else if (flag == step * 9)cout << ".";
	else if (flag == step * 10)cout << ".";
	else if (flag == step * 11)cout << ".";
	else if (flag == step * 12)cout << "30%";
	else if (flag == step * 13)cout << ".";
	else if (flag == step * 14)cout << ".";
	else if (flag == step * 15)cout << ".";
	else if (flag == step * 16)cout << "40%";
	else if (flag == step * 17)cout << ".";
	else if (flag == step * 18)cout << ".";
	else if (flag == step * 19)cout << ".";
	else if (flag == step * 20)cout << "50%";
	else if (flag == step * 21)cout << ".";
	else if (flag == step * 22)cout << ".";
	else if (flag == step * 23)cout << ".";
	else if (flag == step * 24)cout << "60%";
	else if (flag == step * 25)cout << ".";
	else if (flag == step * 26)cout << ".";
	else if (flag == step * 27)cout << ".";
	else if (flag == step * 28)cout << "70%";
	else if (flag == step * 29)cout << ".";
	else if (flag == step * 30)cout << ".";
	else if (flag == step * 31)cout << ".";
	else if (flag == step * 32)cout << "80%";
	else if (flag == step * 33)cout << ".";
	else if (flag == step * 34)cout << ".";
	else if (flag == step * 35)cout << ".";
	else if (flag == step * 36)cout << "90%";
	else if (flag == step * 37)cout << ".";
	else if (flag == step * 38)cout << ".";
	else if (flag == step * 39)cout << ".";
	//cout << "100%" << endl;
}