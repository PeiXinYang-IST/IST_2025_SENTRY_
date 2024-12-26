import yaml
import numpy as np
import open3d as o3d  # 使用 open3d 库处理点云

# 1. 读取 YAML 格式的栅格地图
def load_yaml_map(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)  # 读取 YAML 数据
    print(f"read yaml success")
    return data

# 2. 从 YAML 数据中提取栅格地图路径
def extract_map_path(yaml_data):
    print(f"get image path")
    return yaml_data['image']

# 3. 从 PGM 文件中读取栅格地图数据
def read_pgm_map(map_path):
    # 使用 numpy 读取 PGM 文件
    with open(map_path, 'rb') as f:
        assert f.readline() == b'P5\n'  # 验证 PGM 格式
        
        # 跳过注释行
        while True:
            line = f.readline().decode().strip()
            if not line.startswith('#'):
                break
        
        # 读取宽度、高度和最大值
        width, height = [int(i) for i in line.split()]
        maxval = int(f.readline())
        
        # 读取图像数据
        grid_map = np.frombuffer(f.read(), dtype=np.uint8).reshape(height, width)
    return grid_map

# 4. 将障碍物格转化为点云
def grid_map_to_point_cloud(grid_map, resolution, occupied_thresh):
    point_cloud = []

    for i in range(grid_map.shape[0]):  # 遍历行
        for j in range(grid_map.shape[1]):  # 遍历列
            if grid_map[i, j] <= occupied_thresh:  # 假设障碍物值为 occupied_thresh 或以上
                x = (j - grid_map.shape[1] / 2) * resolution  # x 坐标
                y = (i - grid_map.shape[0] / 2) * resolution  # y 坐标
                z = 0  # 假设障碍物在地面上
                point_cloud.append([x, y, z])
    for i in range(grid_map.shape[0]):  # 遍历行
        for j in range(grid_map.shape[1]):  # 遍历列
            if grid_map[i, j] <= occupied_thresh:  # 假设障碍物值为 occupied_thresh 或以上
                x = (j - grid_map.shape[1] / 2) * resolution  # x 坐标
                y = (i - grid_map.shape[0] / 2) * resolution  # y 坐标
                z = 0.2  # 假设障碍物在地面上
                point_cloud.append([x, y, z])
    for i in range(grid_map.shape[0]):  # 遍历行
        for j in range(grid_map.shape[1]):  # 遍历列
            if grid_map[i, j] <= occupied_thresh:  # 假设障碍物值为 occupied_thresh 或以上
                x = (j - grid_map.shape[1] / 2) * resolution  # x 坐标
                y = (i - grid_map.shape[0] / 2) * resolution  # y 坐标
                z = 0.1  # 假设障碍物在地面上
                point_cloud.append([x, y, z])
    for i in range(grid_map.shape[0]):  # 遍历行
        for j in range(grid_map.shape[1]):  # 遍历列
            if grid_map[i, j] <= occupied_thresh:  # 假设障碍物值为 occupied_thresh 或以上
                x = (j - grid_map.shape[1] / 2) * resolution  # x 坐标
                y = (i - grid_map.shape[0] / 2) * resolution  # y 坐标
                z = 0.3  # 假设障碍物在地面上
                point_cloud.append([x, y, z])

    return np.array(point_cloud)

# 5. 保存点云为 PCD 文件
def save_point_cloud(point_cloud, filename="world.pcd"):
    # 使用 open3d 创建点云对象
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(point_cloud)  # 将点云数据转换为 open3d 格式
    o3d.io.write_point_cloud(filename, cloud)  # 保存为 PCD 文件

# 主程序
def main():
    yaml_file = '/home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/Map/demo2.yaml'  # 输入你的 YAML 文件路径
    pcd_file = '/home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/PCD/world.pcd'  # 输出点云的路径
    yaml_data = load_yaml_map(yaml_file)  # 读取 YAML 文件
    map_path = extract_map_path(yaml_data)  # 提取栅格地图路径
    grid_map = read_pgm_map(map_path)  # 从 PGM 文件中读取栅格地图数据
    resolution = yaml_data['resolution']
    occupied_thresh = yaml_data['occupied_thresh']
    
    # 转换栅格地图中的障碍物格为点云
    point_cloud = grid_map_to_point_cloud(grid_map, resolution, occupied_thresh)
    
    # 保存为 PCD 文件
    save_point_cloud(point_cloud, pcd_file)
    print(f"点云已保存为 '{pcd_file}'")

if __name__ == "__main__":
    main()