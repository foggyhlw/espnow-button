# espnow-button
- 使用esp-12f模块制作的3按钮遥控器，使用esp-now通讯协议
- esp-now数据包格式可以自行修改，需要注意在sender和receiver都要同步修改，保证数据格式完全一致
- 使用时先烧录receiver端程序，连接串口后启动会在串口输出自身mac地址，将该mac地址填入sender的broadcastAddress[]中再烧录sender代码
- 功能介绍和使用详见：
