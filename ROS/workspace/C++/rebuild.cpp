#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>  // 贪婪投影三角化算法

int main() {
    // 1. 加载点云数据（假设已去除背景噪声）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("input.pcd", *cloud);

    // 2. 表面重建配置
    pcl::GreedyProjectionTriangulation<pcl::PointXYZ> gp3;
    pcl::PolygonMesh mesh;  // 存储重建后的网格模型
    gp3.setInputCloud(cloud);
    gp3.setSearchRadius(0.05);    // 搜索邻域半径（需根据点密度调整）
    gp3.setMu(2.5);               // 平滑系数，值越大表面越平滑
    gp3.setMaximumNearestNeighbors(100); // 最大邻域点数
    gp3.reconstruct(mesh);        // 执行重建

    // 3. 保存网格模型（OBJ格式）
    pcl::io::saveOBJFile("output.obj", mesh);
    return 0;
}
