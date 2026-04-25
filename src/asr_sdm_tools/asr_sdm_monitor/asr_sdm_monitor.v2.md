# asr_sdm_monitor.v2

##### 增加功能

+ video_monitor.py最多可以同时监测2个节点，根据ui动态调整监测0-2个/perception * 的topic

+ UI两个窗口默认是None，通过下拉菜单选择需要的/perception*

##### 修改

+ 修改了UI的Warning
+ 在运行过程中发现/diagnotics话题运行5分钟左右会丢失。topic list没输出，node list也没有。然后在.bashrc里加上下面两行，问题就解决了，连续展示2h正常监测。但是不知道这样改.bashrc可以吗。

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

+ 标题栏ASR SDM Monitor随主题颜色改变
+ CPU / Memory / HDD / Net / NTP随页面宽度自动伸缩