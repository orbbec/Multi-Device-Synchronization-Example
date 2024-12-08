# 完成硬件连接和软件配置，[参考文档](https://github.com/orbbec/Multi-Device-Synchronization-Example/blob/main/README_CN.MD#3-%E7%A1%AC%E4%BB%B6%E8%BF%9E%E6%8E%A5%E6%8C%87%E5%8D%97)

# 设置保存图片组数
SAVE_FRAME_COUNT_TARGET = 200；

# 备注
图片保存速度和硬盘速度有关，存在速度限制，建议使用高速固态硬盘。当硬盘速度不足时，会出现丢帧情况，Log显示“Frame Aggregation Queue overflow.”。