#发布名为/agv_gpio的topic
#消息类型为 /gpio
#10是队列长度，这个是用来缓冲来不及发布的数据
#magnetic_front 前磁扫描仪
#magnetic_back  后磁扫描仪
#magnetic_break 精准定位刹车磁扫描仪

Header header
uint16 magnetic_front
uint16 magnetic_back
uint16 magnetic_break
