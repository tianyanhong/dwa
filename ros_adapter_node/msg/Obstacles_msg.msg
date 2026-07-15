#时间戳
time time_stamp
#坐标系 fuse_link:激光和相机融合过后的坐标系
##     front_camera_link:前上相机坐标系
##     back_camera_link:末端定位相机坐标系
##     wj_laser_link:下方万集激光坐标系
string frame_id
#障碍物所在区域：1--外层 2--中层  3--急停
int8 area
#障碍物的信息
Obstacle[] obstacles