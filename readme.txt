FeatureAdapter功能：计算pcd格式点云PFH特征。
使用方法：
1. 下载PCL-1.9.1-AllInOne-msvc2017-win64.exe并安装。
2. 添加环境变量 PCL_ROOT，值为PCL安装路径，如D:\Program Files\PCL 1.9.1
2. 用VS2017打开FeatureAdapter.sln
3. 分别选择Release | x64 和Debug | x64，生成。

运行生成的exe文件，如果出现“未找到OpenNI2.dll”错误提示，下载OpenNI2.dll到生成的exe同级目录下。