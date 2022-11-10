// FeatureAdapter.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iostream>
#include <time.h>
#include "pfh.h"
#include <string.h>
#define MAX_LINE 100 //文件读取一行最大长度
using namespace pcl;
using namespace std;

bool IsXYZCorrect(float _x, float _y, float _z,int _nRef)
{
	if (_x > 30000 || _x < -30000)
		return false;
	if (_y > 30000 || _y < -30000)
		return false;
	if (_z > 10000 || _z < -10000)
		return false;
	if (_nRef > 256 || _nRef < 5)
		return false;
	return true;
}

int main(int argc,char *argv[])
{
	//argv[1] = "e:\\OfficeE\\OfficeE_MatchAdapter4.txt";
	if (argv[1] == NULL)
	{
		cout << "insufficient parameters.";
		return 0;
	}
	//读取MatchAdapter文件
	FILE* pfMatchAdpt;
	fopen_s(&pfMatchAdpt, argv[1], "r");
	if (pfMatchAdpt == NULL)
	{
		cout << "failed to open adapter file.";
		return 0;
	}
	char strBuffer[MAX_LINE];//缓冲区
	char* strPoiPath = new char[50];//中间文件路径
	char* strFinalTieName = new char[50];//最终tie文件名
	int nCloudNum = 0;//点云文件个数
	int len = 0;
	fgets(strPoiPath, MAX_LINE, pfMatchAdpt);
	strPoiPath[strlen(strPoiPath) - 1] = 0;
	fgets(strFinalTieName, MAX_LINE, pfMatchAdpt);
	fgets(strBuffer, 10, pfMatchAdpt);
	nCloudNum = atoi(strBuffer);
	int* nCloudID = new int[nCloudNum];//点云文件id数组
	char** strCloudName = new char*[nCloudNum];//点云文件名数组
	for (int i = 0; i < nCloudNum; ++i)
		strCloudName[i] = new char[MAX_LINE];
	int *nCloudIDIter = nCloudID;
	for (int i = 0; i < nCloudNum; ++i)
	{
		fgets(strBuffer, 5, pfMatchAdpt);
		*nCloudIDIter = atoi(strBuffer);
		nCloudIDIter++;
		fgets(strBuffer, MAX_LINE, pfMatchAdpt);
		strBuffer[strlen(strBuffer) - 1] = 0;
		strcpy(strCloudName[i], strBuffer);
	}
	//去除点云文件名首空格
	for (int i = 0; i < nCloudNum; ++i)
	{
		char *start;
		int len = strlen(strCloudName[i]);
		start = strCloudName[i];
		while (*start && isspace(*start))
			start++;
		strcpy(strCloudName[i], start);
	}
	//读取邻接矩阵
	int **nNeiborMatrix = new int*[nCloudNum];
	for (int i = 0; i < nCloudNum; ++i)
		nNeiborMatrix[i] = new int[nCloudNum];
	int ch;
	int i = 0, j = 0;
	while ((ch = fgetc(pfMatchAdpt)) != EOF)
	{
		if (ch != '\n'&&ch != ' ')
		{
			nNeiborMatrix[i][j] = ch - '0';
			j++;
			if (j == nCloudNum)
			{
				i++;
				j = 0;
			}
		}
	}
	//生成输出poi文件名数组
	char** strPoiName = new char*[nCloudNum];//特征描述poi文件名数组
	for (int i = 0; i < nCloudNum; ++i)
		strPoiName[i] = new char[MAX_LINE];
	char* strPoiPathTemp = new char[50];
	for (int i = 0; i < nCloudNum; ++i)
	{
		strcpy(strPoiPathTemp, strPoiPath);
		char *p = strrchr(strCloudName[i], '\\')+1;
		strcpy(strPoiName[i],strcat(strPoiPathTemp, p));
		strPoiName[i][strlen(strPoiName[i]) - 3] = 'p';
		strPoiName[i][strlen(strPoiName[i]) - 2] = 'o';
		strPoiName[i][strlen(strPoiName[i]) - 1] = 'i';
	}
	//计算特征 保存poi文件
	PointXYZRGB point;
	uint64_t nTimestamp;
	for (int iter = 0; iter < nCloudNum; iter++)
	{
		PointCloud<PointXYZRGB>::Ptr Cloud(new PointCloud<PointXYZRGB>);
		cout << "Calculating:" << strCloudName[iter] << "  ";
		//io::loadPCDFile(strCloudName[iter], *Cloud);
		FILE* pfCloudFile;
		fopen_s(&pfCloudFile, strCloudName[iter], "r");
		if (pfCloudFile == NULL)
		{
			cout << "failed to load cloud file.";
			return 0;
		}
		char strLine[50];
		while (!feof(pfCloudFile))
		{
			fgets(strLine, 50, pfCloudFile);
			sscanf(strLine, "%f,%f,%f,%d,%lld,\n", &point.x, &point.y, &point.z, &point.rgb, &nTimestamp);
			point.x *= 1000;
			point.y *= 1000;
			point.z *= 1000;  
			if (IsXYZCorrect(point.x, point.y, point.z, point.rgb))
				Cloud->push_back(point);
		}
		PFH* CloudPFH;
		PointCloud<Normal>::Ptr FeatureNormal(new PointCloud<Normal>);
		PointCloud<PointXYZRGB>::Ptr Feature(new PointCloud<PointXYZRGB>);
		Feature = CloudFeatureExtraction(Cloud, FeatureNormal, 900, 5);
		CloudPFH = CalculatePFH(Feature, FeatureNormal, 2500);
		FILE* pfPFH;
		fopen_s(&pfPFH, strPoiName[iter], "w");
		if (pfPFH == NULL)
		{
			cout << "failed to open poi file.";
			return 0;
		}
		fprintf(pfPFH,"%d %d\n",Feature->size(), nCloudID[iter]);//文件头写入特征点数，文件id
		for (int i = 0; i < Feature->size(); ++i)
		{
			fprintf(pfPFH, "(%.4f,%.4f,%.4f,%.0f):", CloudPFH->x / 1000 , CloudPFH->y / 1000, CloudPFH->z / 1000, CloudPFH->ref);
			for (int j = 0; j < 125; j++)
				fprintf(pfPFH, "%.4f ", CloudPFH->fHistogram[j]);
			fprintf(pfPFH, "\n");
			CloudPFH++;
		}
		fclose(pfPFH);
		cout << "100%" << endl;
		fclose(pfCloudFile);
	}
	//清理
	fclose(pfMatchAdpt);
	delete[] strPoiPath;
	strPoiPath = NULL;
	delete[] strFinalTieName;
	strFinalTieName = NULL;
	delete[] nCloudID;
	nCloudID = NULL;
	for (int i = 0; i < nCloudNum; ++i)
	{
		delete[] strCloudName[i];
		strCloudName[i] = NULL;
	}
	for (int i = 0; i < nCloudNum; ++i)
	{
		delete[] nNeiborMatrix[i];
		nNeiborMatrix[i] = NULL;
	}
	for (int i = 0; i < nCloudNum; ++i)
	{
		delete[] strPoiName[i];
		strPoiName[i] = NULL;
	}
	delete[] strPoiPathTemp;
	strPoiPathTemp = NULL;
    return 0;
}

