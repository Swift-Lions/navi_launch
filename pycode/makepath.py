# import open3d as o3d
# import numpy as np


# # 🔧 이동 비용 함수: x/y 이동은 비싸고 z 이동은 저렴
# def custom_movement_cost(dx, dy, dz, xy_weight=1.0, z_weight=0.3):
#     return np.sqrt((dx**2 + dy**2) * xy_weight + (dz**2) * z_weight)

# # 1. PCD 불러오기
# pcd = o3d.io.read_point_cloud("/home/yoon/unitree_ros2/navi_ws/voxel_grid_output.pcd")
# points = np.asarray(pcd.points)

# # 2. Occupancy Grid로 변환
# voxel_size = 0.2
# min_bound = np.min(points, axis=0)
# voxel_indices = np.floor((points - min_bound) / voxel_size).astype(int)

# # 3. Occupied voxel 기록
# occupied = set(tuple(v) for v in voxel_indices)

# # 4. Start/Goal voxel 설정
# def point_to_voxel(p):
#     return tuple(np.floor((np.array(p) - min_bound) / voxel_size).astype(int))

# start_voxel = point_to_voxel([0, 0, 0])            # 예시 시작점
# goal_voxel = point_to_voxel([10, 0, 0.0])       # 예시 목표점

# # 5. 3D A* 경로 탐색
# from queue import PriorityQueue

# def astar_3d(start, goal, occupied):
#     neighbors = [(i, j, k) for i in [-1,0,1] for j in [-1,0,1] for k in [-1,0,1] if not (i==j==k==0)]
#     frontier = PriorityQueue()
#     frontier.put((0, start))
#     came_from = {start: None}
#     cost_so_far = {start: 0}

#     while not frontier.empty():
#         _, current = frontier.get()

#         if current == goal:
#             break

#         for dx, dy, dz in neighbors:
#             next_voxel = (current[0]+dx, current[1]+dy, current[2]+dz)
#             if next_voxel in occupied:
#                 continue
#             #new_cost = cost_so_far[current] + custom_movement_cost(dx, dy, dz)

#             new_cost = cost_so_far[current] + np.linalg.norm([dx,dy,dz])
#             if next_voxel not in cost_so_far or new_cost < cost_so_far[next_voxel]:
#                 cost_so_far[next_voxel] = new_cost
#                 priority = new_cost + np.linalg.norm(np.array(goal)-np.array(next_voxel))
#                 frontier.put((priority, next_voxel))
#                 came_from[next_voxel] = current

#     # 경로 역추적
#     path = []
#     cur = goal
#     while cur != start:
#         path.append(cur)
#         cur = came_from.get(cur)
#         if cur is None:
#             return []  # 경로 없음
#     path.append(start)
#     path.reverse()
#     return path

# voxel_path = astar_3d(start_voxel, goal_voxel, occupied)

# # 6. 경로를 실제 좌표로 변환
# real_path = [min_bound + (np.array(v) + 0.5) * voxel_size for v in voxel_path]

# # # 7. 시각화
# # path_pcd = o3d.geometry.PointCloud()
# # path_pcd.points = o3d.utility.Vector3dVector(real_path)
# # path_pcd.paint_uniform_color([1.0, 0.0, 0.0])  # 경로는 빨간 점

# # o3d.visualization.draw_geometries([pcd, path_pcd])
# # 1. 기존 맵 로드 (회색 등으로)
# pcd = o3d.io.read_point_cloud("voxel_grid_output.pcd")
# #pcd.paint_uniform_color([0.6, 0.6, 0.6])  # 회색



# # 2. z값 추출
# points = np.asarray(pcd.points)
# z_vals = points[:, 2]

# # 3. z값 정규화 (0 ~ 1)
# z_min = z_vals.min()
# z_max = z_vals.max()
# z_norm = (z_vals - z_min) / (z_max - z_min + 1e-5)

# # 4. 색상 매핑 (예: 높을수록 더 빨갛게)
# colors = np.stack([z_norm, 1.0 - z_norm, 0.5 * np.ones_like(z_norm)], axis=1)
# # 색 예: (z, 1-z, 0.5) → 노란-초록-빨강 그라디언트 느낌

# # 5. 색 적용
# pcd.colors = o3d.utility.Vector3dVector(colors)

# # 6. 시각화 (경로 없이 맵만 표시)
# o3d.visualization.draw_geometries([pcd])



# # 2. 경로를 PointCloud로 생성 (빨간색)
# path_pcd = o3d.geometry.PointCloud()
# path_pcd.points = o3d.utility.Vector3dVector(real_path)
# path_pcd.paint_uniform_color([1.0, 0.0, 0.0])  # 빨간색

# # 3. 같이 시각화

# o3d.visualization.draw_geometries(
#     [pcd, path_pcd],
#     zoom=0.5,
#     front=[0.0, -1.0, 0.0],
#     lookat=[0, 0, 0],
#     up=[0.0, 0.0, 1.0]
# )
import open3d as o3d
import numpy as np
from queue import PriorityQueue

# 🔧 이동 비용 함수: x/y 이동은 비싸고 z 이동은 저렴
def custom_movement_cost(dx, dy, dz, xy_weight=1.0, z_weight=0.3):
    return np.sqrt((dx**2 + dy**2) * xy_weight + (dz**2) * z_weight)

# 1. PCD 불러오기
pcd = o3d.io.read_point_cloud("/home/yoon/unitree_ros2/navi_ws/GlobalMap 0508.pcd")
points = np.asarray(pcd.points)
print("dajflasdjf")

# 2. Occupancy Grid로 변환
voxel_size = 0.2
min_bound = np.min(points, axis=0)
voxel_indices = np.floor((points - min_bound) / voxel_size).astype(int)

# 3. Occupied voxel 기록 (z < 0 인 것도 막힘으로 간주)
#occupied = set()
# for idx, pt in zip(voxel_indices, points):
#     if pt[2] < 0:  # z < 0 이면 차단
#         occupied.add(tuple(idx))
#     else:
#         occupied.add(tuple(idx))
# 3. Occupied voxel 기록 (z < 0 인 것만 막힌 것으로 간주)
occupied = set()
for idx, pt in zip(voxel_indices, points):
    if pt[2] < 0:  # 🔴 여기만 occupied로 추가
        occupied.add(tuple(idx))


# 4. Start/Goal voxel 설정
def point_to_voxel(p):
    return tuple(np.floor((np.array(p) - min_bound) / voxel_size).astype(int))

start_voxel = point_to_voxel([0, 0, 0])
goal_voxel = point_to_voxel([10, 0, 0.0])

# 5. A* 구현
def astar_3d(start, goal, occupied):
    print("Astar start")
    neighbors = [(i, j, k) for i in [-1,0,1] for j in [-1,0,1] for k in [-1,0,1] if not (i==j==k==0)]
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        _, current = frontier.get()

        if current == goal:
            break

        for dx, dy, dz in neighbors:
            next_voxel = (current[0]+dx, current[1]+dy, current[2]+dz)
            if next_voxel in occupied:
                continue
            new_cost = cost_so_far[current] + np.linalg.norm([dx,dy,dz])
            if next_voxel not in cost_so_far or new_cost < cost_so_far[next_voxel]:
                cost_so_far[next_voxel] = new_cost
                priority = new_cost + np.linalg.norm(np.array(goal)-np.array(next_voxel))
                frontier.put((priority, next_voxel))
                came_from[next_voxel] = current

    # 경로 역추적
    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = came_from.get(cur)
        if cur is None:
            return []  # 경로 없음
    path.append(start)
    path.reverse()
    return path

voxel_path = astar_3d(start_voxel, goal_voxel, occupied)
real_path = [min_bound + (np.array(v) + 0.5) * voxel_size for v in voxel_path]

# 🔹 1.5m 간격마다 위치를 저장 (처음은 포함)
sampled_path = [real_path[0]]
dist_acc = 0.0
for i in range(1, len(real_path)):
    dist_acc += np.linalg.norm(np.array(real_path[i]) - np.array(real_path[i-1]))
    if dist_acc >= 1.5:
        sampled_path.append(real_path[i])
        dist_acc = 0.0

# 🔸 높이 기반 색상 입히기
pcd = o3d.io.read_point_cloud("/home/yoon/unitree_ros2/navi_ws/voxel_grid_output.pcd")
points = np.asarray(pcd.points)
z_vals = points[:, 2]
z_min, z_max = z_vals.min(), z_vals.max()
z_norm = (z_vals - z_min) / (z_max - z_min + 1e-5)
colors = np.stack([z_norm, 1.0 - z_norm, 0.5 * np.ones_like(z_norm)], axis=1)
pcd.colors = o3d.utility.Vector3dVector(colors)

# 🔴 경로 (빨간 점)
path_pcd = o3d.geometry.PointCloud()
path_pcd.points = o3d.utility.Vector3dVector(real_path)
path_pcd.paint_uniform_color([1.0, 0.0, 0.0])

# 🔵 1.5m 간격 경유지 (파란 점)
sampled_pcd = o3d.geometry.PointCloud()
sampled_pcd.points = o3d.utility.Vector3dVector(sampled_path)
sampled_pcd.paint_uniform_color([0.0, 0.0, 1.0])

# 📷 시각화
o3d.visualization.draw_geometries(
    [pcd, path_pcd, sampled_pcd],
    zoom=0.5,
    front=[0.0, -1.0, 0.0],
    lookat=[0, 0, 0],
    up=[0.0, 0.0, 1.0]
)
