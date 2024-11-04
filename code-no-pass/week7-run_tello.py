#!/usr/bin/env python
# -*- coding: utf-8 -*-

### 範例程式 ###
### 使用simple_tello.py 來建立Tello與控制Tello執行相應動作 ###

import rospy                         # 導入套件: rospy
import simple_tello                  # 導入 simple_tello
from geometry_msgs.msg import Twist  # 導入 geometry_msgs 裡的 Twist 訊息格式
from std_msgs.msg import Empty       # 導入 std_msgs 裡的 Empty 訊息格式

# main
# 負責執行一連串的指令
def main():
  t1 = simple_tello.Tello_drone()    #建立 simple_tello.Tello_drone 變數 t1
  
  while t1.state.is_flying == False: # 當is_flying為False, 表示無人機還沒有成功起飛, 故透過while 迴圈不斷執行起飛, 直到is_flying為True, 表示無人機起飛成功
    t1.controler.takeoff()
  
  while t1.state.fly_mode != 6:      # 當fly_mode != 6, 表示無人機正在執行特殊行為, 如 起飛 降落 翻滾等等, 因此透過while迴圈確認特殊行為是否結束
    print("wait...")
  
  t1.controler.flip(0)               # 使用 t1 中 controler 裡的 flip function, 讓無人機進行翻滾, 電量不夠不會翻
   
  while t1.state.fly_mode != 6:      # 當fly_mode != 6, 表示無人機正在執行特殊行為, 如 起飛 降落 翻滾等等, 因此透過while迴圈確認特殊行為是否結束
    print("wait...") 
    
  while t1.state.is_flying == True:  # 當is_flying為True, 表示無人機還正在飛行, 故透過while 迴圈不斷執行降落, 直到is_flying為False, 表示無人機完成降落
    t1.controler.land() 

# main  
if __name__ == "__main__":

  # 告訴ros此程式為node, node名稱為 'run_tello', anonymous為True,表示ros會幫此node加一段亂碼進行匿名
  rospy.init_node("run_tello", anonymous=True)
  
  #執行 範例function main
  main()
