# zllens Arduino下的设计

文件名 | 函数、状态等等 | 设计
---|--- |---
zllens.ino | fsm，loop， setup | 获取payload，分为state sensitive的，或者每个命令下都改状态。
cmd_ack.cpp | cmd_2_mean, mean_2_ack | 命令解析到“数据结构”
calc_afer.cpp | reset_2_new_f, save_lap_2_new_f_and_is_end, get_best_f_lap |帮助计数用
package.cpp | recv_payload(p,len), send_payload(ack, len)|串口的
lens_motor.cpp | loop_5ms(), set(), get(), find_jump(), set_cur()|有loop，有状态，有初始化。



