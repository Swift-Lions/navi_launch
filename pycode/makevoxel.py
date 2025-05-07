import open3d as o3d
import numpy as np

# 📥 1. PCD 파일 불러오기
pcd = o3d.io.read_point_cloud("/home/yoon/Desktop/GlobalMap5.pcd")  # 경로 확인
points = np.asarray(pcd.points)

# ⚙️ 2. Voxel grid 설정
voxel_size = 0.2
min_bound = np.min(points, axis=0)
voxel_indices = np.floor((points - min_bound) / voxel_size).astype(int)

# 🧮 3. 중복 제거된 voxel 중심 좌표 계산
voxel_set = set()
voxel_centers = []
for idx in voxel_indices:
    key = tuple(idx)
    if key not in voxel_set:
        voxel_set.add(key)
        center = min_bound + (np.array(key) + 0.5) * voxel_size
        voxel_centers.append(center)

voxel_centers = np.array(voxel_centers)

# 🔲 4. 원점 추가
origin = np.array([[0.0, 0.0, 0.0]])
all_points = np.vstack([voxel_centers, origin])

# 🎨 5. 색상 설정 (기본 흰색, 마지막 점 = 원점 = 검정색)
z_values = all_points[:, 2]
norm_z = (z_values - z_values.min()) / (z_values.max() - z_values.min() + 1e-5)
colors = np.stack([norm_z, norm_z, norm_z], axis=1)  # 회색조
colors[-1] = [0, 0, 0]



# 🧱 6. PointCloud 생성
new_pcd = o3d.geometry.PointCloud()
new_pcd.points = o3d.utility.Vector3dVector(all_points)
new_pcd.colors = o3d.utility.Vector3dVector(colors)


print(np.asarray(new_pcd.colors)[-5:])  # 마지막 5개 점 색상 확인


# 👀 7. 시각화 (점 크기 크게)
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(new_pcd)

opt = vis.get_render_option()
opt.point_size = 10.0  # 점 크게

vis.run()
vis.destroy_window()

# 💾 8. 저장
o3d.io.write_point_cloud("voxel_grid_output.pcd", new_pcd)

