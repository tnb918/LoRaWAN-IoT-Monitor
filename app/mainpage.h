/**
* @mainpage 介绍
* @author     Lierda-WSN-LoRaWAN-MT
* @version V1.1.6
* @date 2019-07-23
*
* LoRaWAN_ICA_Node_Driver 是针对 **ICA模块** 的驱动程序，驱动程序主要通过串口与GPIO与模块交互，用户可将该驱动移值到实际的应用程序中，以便快速接入 **LinkWAN** 平台，目前驱动提供以下功能：
* - 1. ICA模块工作模式与工作状态控制接口
* 	- 激活状态与休眠状态切换
* 	- 指令与透传模式的切换
* - 2. AT指令进行模块参数配置接口
* - 3. 入网接口
* - 4. 数据发送接口
* - 5. 大数据包发送接口（驱动中已实现终端的拆分包协议）
*
* @section application_arch 01 模块驱动的应用框架
*
* LoRaWAN ICA Node Driver典型使用方式：
* @image html LoRaWAN_ICA-Module-driver.png "Figure 1: Uart driver of ICA Node
*
* @section  LoRaWAN_ICA_Node_Driver API
* 更多详细信息，请参考 @ref ICA-Driver
*/


