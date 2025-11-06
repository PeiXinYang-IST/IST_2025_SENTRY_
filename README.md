
主要规划算法在Gcopter+中

1.前端使用A*搜索

2.获取初始路径后使用ESDF检测碰撞并推离得到dealed path

3.通过dealed path在二维平面上拓展生成二维安全走廊（一种朴素的通过esdf的碰撞检测的生成方法 time<1ms）

4.构建约束条件使用minco规划轨迹

5.将返回的五次多项式将位置项作为NMPC跟踪目标 实时replan信息（0.01s向轨迹forward急刹）

![运行示意图](https://github.com/PeiXinYang-IST/IST_2025_SENTRY_/blob/main/2025-11-06%2023-46-14%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png) 

实物部署视频： https://www.bilibili.com/video/BV1RULizJEYn/?spm_id_from=333.1387.homepage.video_card.click

https://www.bilibili.com/video/BV1Yj5RzcEJo/?spm_id_from=333.1387.homepage.video_card.click




