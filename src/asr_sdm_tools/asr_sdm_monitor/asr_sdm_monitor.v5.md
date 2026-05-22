# asr_sdm_monitor.v5

增加

+ Plot页面(以IMU的速度和加速度数据为例)
  + Plot有一个自定义的候选topic集合，当检测到有候选topic正在发布，就可以在Plot里可视化
  + Plot曲线支持鼠标滚轮缩放，左键拖动和Reset恢复
  + Live可视化当前的topic
    + Start Recording可以记录当前的topic
  + Recorded播放ros2bag
    + Open打开文件目录
    + Play播放/Pause停止
    + Star/End/Current time可以在允许范围内自定义，Current time可以在进度条手动调整
    + Speed播放速度
  + X轴
    + Lable type决定X轴数据类型，可选Time或Topic message；当X轴Lable type选Topic message，如IMU测量的x轴的速度，若Y轴选择IMU测量的y轴的速度，则绘制xy平面上的速度
    + Timestamp format可选相对时间和绝对时间
    + Time window表示窗口展示的topic时长，默认4s，即默认展示Current time前后2s的数据
    + Label下拉菜单选择topic
    + Show tick labels是否展示刻度标签
  + Y轴
    + Series number默认1个，最多16个Serises
    + Show tick labels是否展示刻度标签
    + Label下拉菜单选择topic，相同的topic不可以重复选择
    + 每个Series独立选择Label(topic)/Color/Line width
+ Hardware/Video/Plot收起选项