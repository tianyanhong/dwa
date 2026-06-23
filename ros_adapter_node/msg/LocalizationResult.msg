# 定位结果的标识符
# * EKF定位        base_link_laser_E 
# * A激光定位      base_link_laser_A   该点一般与运动中心重合(内部使用)
# * B激光定位      base_link_laser_B   该点一般与运动中心重合(内部使用)
# * 激光定位       base_link_laser     该点一般与运动中心重合(发布出去的)
# * 二维码定位     base_link_QR        该点一般与二维码相机重合
# * 末端定位       base_link_detector  该点一般与运动中心重合
string frame
 
# 定位结果所在的参考坐标系，目前有如下几种
# * 全局坐标系                   global_link
# * 二维码坐标系                 QR_code_xxx  (xxx为具体识别出来的二维码编号)
# * 末端定位参考的车体坐标系     base_link_detector
string ref_frame
 
#定位结果时间      一般为激光扫描结束时间
time time_stamp
#定位结果的心跳id  定位结果更新一次，该值加一，初始化为0
uint32 seq_id
# 定位结果状态     -2表示未初始化，-1表示定位丢失，1表示定位质量较低，2表示结果正常定位
int32 loc_state
# 定位结果置信度       0~1，越高代表定位结果越优
float32 certainty
# 定位结果和协方差矩阵
geometry_msgs/PoseWithCovariance pose_with_covariance

