#!/usr/bin/env python2
# -*- coding: utf-8 -*-

### 本程式為影像處理迴圈範例程式, 用於從Tello的影像中進行HSV過濾, 找出特定的顏色mask, 並把mask顯示出來 ###

import rospy                                 # 導入套件: rospy
from sensor_msgs.msg import CompressedImage  # 導入 sensor_msgs 裡的 CompressedImage
import av                                    # 導入套件: av
import cv2                                   # 導入套件: cv2
import numpy as np                           # 導入套件: numpy 並命名成 np
import threading                             # 導入套件: threading
import traceback                             # 導入套件: traceback

### class StandaloneVideoStream 
### 用於將訂閱/tello/image_raw/h264所獲得的 CompressedImage 進行處理, 得到stream
### 再從stream中取得影像
class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()

# 建立 stream
stream = StandaloneVideoStream()

# 定義 function callback, 訂閱 "/tello/image_raw/h264", 用於處理接收到的compressedImg, (msg)請保留, msg即為收到的用於處理接收到的compressedImg()
def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data)) # 可以解開註解來觀看目前收到的bytes穩不穩定
    # 將接收到的compressedImg加入到stream中
    stream.add_frame(msg.data)

# 定義function: findMask, 輸入為cv hsv_img(HSV格式), 用途為根據給定的HSV 上下界值域進行過濾 從影像中找出特定顏色的mask 
def find_Mask(img):

  # 定義 HSV 值域的下界, 詳見投影片中 HSV 範圍圖中的左紅框
  lr0 = np.array([0, 70, 0])
  # 定義 HSV 值域的上界, 詳見投影片中 HSV 範圍圖中的左紅框
  ur0 = np.array([5, 255, 255])
  
  # 定義 HSV 值域的下界, 詳見投影片中 HSV 範圍圖中的右紅框
  lr1 = np.array([175, 70, 0])
  # 定義 HSV 值域的上界, 詳見投影片中 HSV 範圍圖中的右紅框
  ur1 = np.array([180, 255, 255])
  
  # 透過第一組左紅框的HSV值域進行影像過濾
  rm0 = cv2.inRange(img, lr0, ur0)
  
  # 透過第二組右紅框的HSV值域進行影像過濾
  rm1 = cv2.inRange(img, lr1, ur1)
  
  # 將第一組, 第二組紅框值域進行or運算, 將兩者結果合併
  rm = cv2.bitwise_or(rm0, rm1)
  return rm # 回傳結果

# main function
def main():

    # fourcc: video的編碼格式, 如 XVID, MP4V 等等...
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    # out: 建立 VideoWriter, video名稱為 test.avi, 寫入格式為 'X',"V",'I','D', FPS 為 20.0, video解析度為 (影像寬, 影像高)
    # 由於這邊會輸出左影像為原始影像, 右影像為視覺處理的影像, 因此在寬上面會是960*2 = 1920, 高720不變
    out = cv2.VideoWriter('test.avi', fourcc, 20.0, (1920, 720))
    
    # 告訴ros此程式為node, node名稱為 'h264_listener'
    rospy.init_node('h264_listener')
    
    # 定義 rospy.Subscriber, 會訂閱 topic: '/tello/image_raw/h264' , CompressedImage為 topic: '/tello/image_raw/h264' 所需要的訊息格式
    # callback 為 接收訊息與處理function的名字, 可以自行定義, 但需一致
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    
    # 透過 套件 av 來開啟stream
    container = av.open(stream)
    
    # 在視窗上顯示 log: 'main: opened'
    rospy.loginfo('main: opened')
    
    # 對container進行解碼, 獲取已被加入至stream中的影像
    # 為 for loop
    # 當container解碼後發現沒有影像將會終止迴圈, 也代表 此程式會結束執行
    for frame in container.decode(video=0):
        
        # 透過 np.array 將 frame 先轉成np格式, 再透過cvtColor 將RGB格式轉換成BGR格式
        image = cv2.cvtColor(np.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        
        # 使用cvtColor 將 BGR 格式轉換成 HSV 格式
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 執行 function: findMask, 將 hsv_img 作為 參數輸入, red_mask為 HSV 過濾後的結果
        red_mask = find_Mask(hsv_img)
        
        # 對找出的HSV mask 進行輪廓搜尋: findContours
        # findContours 第一個輸入參數為 二值化的mask, 此處可以直接使用我們先前進行HSV過濾的結果
        # findContours 第二個輸入參數為 Flag, 可以設定要找最外圍的輪廓 或是連內部輪廓也一起尋找, 此處使用 RETR_EXTERNAL: 找最外圍的輪廓
        # findContours 第三個輸入參數為 亦為Flag, 可以設定要找全部的點或是僅找角點, 此處使用 CHAIN_APPROX_NONE: 尋找全部的點
        # findContours 會因為cv的版本而有不同的回傳, 如此處有發生問題,　請改成　　_, c_c, contour_h = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        c_c, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # 建立變數 show_image, 用於顯示找到的輪廓與其中心點
        # 首先透過 np.zeros建立空的array, shape為讀取影像的大小, 種類為np.uint8
        # 再來透過 cv2.cvtColor, 將空array 從 gray_scale 格式轉換成 BGR 格式
        show_image = cv2.cvtColor(np.zeros(image.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)
        
        # 透過 drawContours, 將 找到的 c_c 畫在 show_image上
        # -1: 表示目標為 c_c 裡面所有的 contour, 如給 0 表示只畫 第一個
        # (0, 0, 255): contour的顏色, 此處為 BGR 格式, 每一個值的範圍從0 ~ 255, 0最低, 255最大
        # -1: 除了-1以外的數值表示畫輪廓線大小, -1 表示 塗滿
        cv2.drawContours(show_image, c_c, -1, (0, 0, 255), -1)
        
        # 先透過 np.concatenate 將原始影像 image 以及視覺處理的影像 show_image 進行左右合併
        # 將合併後的影像 寫入到video: out中
        out.write(np.concatenate((image, show_image), axis = 1))    
        
        # 顯示 影像, 視窗名稱為result, 欲顯示的cv_img為 np.concatenate((image, show_image), axis = 1)
        cv2.imshow('result', np.concatenate((image, show_image), axis = 1))
        
        # 設定視窗刷新頻率
        cv2.waitKey(1)

# main
if __name__ == '__main__':

    # try catch
    try:
        # 執行 main fuction, 啟動影像處理迴圈
        main()
     
    # 例外處理: BaseException, 有發生會透過 traceback 顯示例外錯誤
    except BaseException:
        traceback.print_exc()
    
    # 最終處理: 關掉 stream, 並將cv2所產生的影像視窗都關掉
    finally:
        stream.close()
        cv2.destroyAllWindows()
