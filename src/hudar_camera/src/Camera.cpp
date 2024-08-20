/*
 * @Author: AlanLee
 * @Date: 2024-07-01 13:26:50
 * @LastEditors: xuxin lisian_magic@163.com
 * @LastEditTime: 2024-08-20 16:28:40
 * @FilePath: /hudar_verge/src/hudar_camera/src/Camera.cpp
 * @Description:
 */
#include "../include/Camera.h"

HKCam::HKCam() {

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    exit(-1);
  }
  if (stDeviceList.nDeviceNum > 0) {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      printf("[device %d]:\n", i);
      MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (NULL == pDeviceInfo) {
        break;
      }
      PrintDeviceInfo(pDeviceInfo);
    }
  } else {
    printf("Find No Devices!\n");
    exit(-2);
  }
  // 选取第一个相机
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
  if (MV_OK != nRet) {
    printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    exit(-3);
  }

  // 打开设备
  // open device
  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    exit(-4);
  }
  // BayerRG8
  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x01080009);
  if (MV_OK == nRet) {
    printf("set PixelFormat OK!\n\n");
  } else {
    printf("set PixelFormat failed! nRet [%x]\n\n", nRet);
  }
  // 设置曝光
  nRet = MV_CC_SetFloatValue(handle, "ExposureTime", 6000);
  if (MV_OK == nRet) {
    printf("set exposure time OK!\n\n");
  } else {
    printf("set exposure time failed! nRet [%x]\n\n", nRet);
  }
  // 设置增益
  nRet = MV_CC_SetFloatValue(handle, "Gain", 15);
  if (MV_OK == nRet) {
    printf("set Gain  OK!\n\n");
  } else {
    printf("set Gain failed! nRet [%x]\n\n", nRet);
  }

  // 设置触发模式为off
  // set trigger mode as off
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
    exit(-5);
  }

  // 开始取流
  // start grab image
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
    exit(-5);
  }

  // ch:获取数据包大小 | en:Get payload size

  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    exit(-6);
  }

  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData) {
    exit(-7);
  }
}
/**
 * @description: 销毁相机句柄
 * @return {*}
 */
HKCam::~HKCam() {
  free(pData);
  // 停止取流
  // end grab image
  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    exit(-10);
  }

  // 关闭设备
  // close device
  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
    exit(-11);
  }

  // 销毁句柄
  // destroy handle
  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    exit(-12);
  }

  if (handle != NULL) {
    MV_CC_DestroyHandle(handle);
    handle = NULL;
  }
}

cv::Mat HKCam::getFrame() {

  unsigned int nDataSize = stParam.nCurValue;

  nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
  if (nRet != MV_OK) {
    printf("No data[%x]\n", nRet);
  }
  cv::Mat res =
      cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);
  cv::Mat bgr = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3);
  // 最后转换成BGR格式
  cv::cvtColor(res, bgr, cv::COLOR_BayerRG2RGB);
  return bgr;
}

bool HKCam::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {

  if (NULL == pstMVDevInfo) {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    int nIp1 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user
    // defined name
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
  } else {
    printf("Not support.\n");
  }

  return true;
}
