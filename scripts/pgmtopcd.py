import numpy as np
import pcl
from PIL import Image

def pgm_to_pcd(pgm_file, pcd_file, resolution=0.05):
    # PGM 파일 로드
    img = Image.open(pgm_file)
    img = np.array(img)
    
    points = []
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            if img[y, x] == 0:  # 점유된 셀
                z = 0  # 2D 맵이므로 z는 0
                points.append([x * resolution, y * resolution, z])
    
    # 포인트 클라우드 생성
    cloud = pcl.PointCloud(np.array(points, dtype=np.float32))
    
    # PCD 파일 저장
    pcl.save(cloud, pcd_file)

# 변환 수행
pgm_to_pcd('/home/heven/catkin_ws/src/map/maps/occupancy_grid.pgm', '/home/heven/catkin_ws/src/map/maps/occupancy_grid.pcd')
