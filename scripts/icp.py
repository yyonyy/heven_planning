# import pcl

# def apply_icp(source_file, target_file):
#     # PCD 파일 로드
#     cloud_source = pcl.load(source_file)
#     cloud_target = pcl.load(target_file)
    
#     # ICP 적용
#     icp = cloud_source.make_IterativeClosestPoint()
#     converged, transf, estimate, fitness = icp.icp(cloud_target, cloud_source)
    
#     # 결과 출력
#     print('Has converged:', converged)
#     print('Fitness score:', fitness)
#     print('Transformation matrix:')
#     print(transf)

# # ICP 적용
# apply_icp('/home/heven/catkin_ws/src/map/maps/occupancy_grid.pcd', '/home/heven/catkin_ws/src/map/maps/GlobalMap.pcd')


import open3d as o3d
import numpy as np

def apply_icp(source_file, target_file):
    source = o3d.io.read_point_cloud(source_file)
    target = o3d.io.read_point_cloud(target_file)

    # 초기 변환 행렬 (단위 행렬로 시작)
    trans_init = np.eye(4)

    # ICP 파라미터 설정
    threshold = 1.0
    icp_result = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    
    print('Has converged:', icp_result.convergence)
    print('Fitness score:', icp_result.fitness)
    print('Transformation matrix:')
    print(icp_result.transformation)
    
    return icp_result.transformation

transformation = apply_icp('/home/heven/catkin_ws/src/map/maps/occupancy_grid.pcd', '/home/heven/catkin_ws/src/map/maps/GlobalMap.pcd')