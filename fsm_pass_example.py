#!/usr/bin/env python
# -*- coding: utf-8 -*-

### 此為有限狀態機FSM範例程式 ### 
### 該程式針對先前的pass_example.py 使用有限狀態機進行修改, ### 
### 該程式針對無人機過框的過程區分為4個狀態 ###
### 定義了各個狀態之間的轉移鏈 ###
### 請搭配先前的 pass_example中的 test_h264_sub.py 以及 simple_tello.py 使用 ###
### 建議在 python3 下使用 ###

import rospy                                  # 導入套件: rospy
import simple_tello                           # 導入 simple_tello

from geometry_msgs.msg import Twist           # 導入 geometry_msgs 裡的 Twist 訊息格式
from std_msgs.msg import Empty                # 導入 std_msgs 裡的 Empty 訊息格式
from statemachine import StateMachine, State  # 導入 statemachine 裡的 StateMachine, State
from time import sleep                        # 導入 time 裡的 sleep

#建立 simple_tello.Tello_drone 變數 t1
t1 = simple_tello.Tello_drone()

### class sMachine
### 建立過框有限狀態機
### 狀態分為 hover (懸停), correction(校正), forward(向前), addSp(加速通過)
### 初始狀態為 hover
### 轉移鏈為 
###     wait4data: hover > hover
###       stop2do: addSp > hover
### start2correct: hover > correction
###  need2correct: correction > correction, forward > correction
### start2forawrd: hover > forward
###  need2forawrd: correction > forward, forward > forward
###    need2addSp: forward > addSp, correction > addSp
### 透過 on_enter 在進入狀態時進行控制行為

class sMachine(StateMachine):

    # state define
    hover = State('Hover', initial = True)
    correction = State('Correction')
    forward = State('Forward')
    addSp = State('AddSp')

    # trans define
    wait4data = hover.to(hover) 
    stop2do = addSp.to(hover)
    
    start2correct = hover.to(correction)
    need2correct = correction.to(correction) | forward.to(correction)

    start2forawrd = hover.to(forward)
    need2forawrd = correction.to(forward) | forward.to(forward)

    need2addSp = forward.to(addSp) | correction.to(addSp)

    # 進入到 hover 狀態時
    def on_enter_hover(self):
      
      # 建立 Twist 訊息, 以 預設值 0 發送 0.5秒 
      msg = Twist()
      t1.controler.move(msg, 0.5)

    # 進入到 correction 狀態時 
    def on_enter_correction(self):
      
      # 建立 Twist 訊息
      msg = Twist()
      
      # 計算 target_x 與 中心點x 480 之間的差
      dx = t1.state.target_x - 480
      
      # 計算 target_y 與 中心點y 200 之間的差
      dy = t1.state.target_y - 200
      
      # 當dx (target_x 與 480的差) 不等於0時, 根據其方向進行左右校正, linear.x: 左- 右+, dx / abs(dx) 表示取其正負號, 亦可用np.sign來取得   
      # 480 為影像中心點 x, 大於 480 表示在中心點的右邊, 小於 480表示在中心點的左邊, 因此當target_x - 480後, +表示要往右邊走, -表示要往左邊走
      if dx != 0:
        msg.linear.x = dx / abs(dx) * 0.1
      
      # 當dy (target_y 與 200的差) 不等於0時, 根據其方向進行上下校正, linear.z: 下- 上+, dy / abs(dy) 表示取其正負號, 亦可用np.sign來取得   
      # 200 為影像中心點 y, 大於 200 表示在中心點的下方, 小於 200表示在中心點的上方, 因此當target_y - 200後, +表示要往下方走, -表示要往上方走, 但這與linear.z的方向相反,因此需要加一個負號來使其一致    
      if dy != 0:
        msg.linear.z = -dy / abs(dy) * 0.2
      
      # 依照設定好的方向與速度持續發送0.5秒
      t1.controler.move(msg, 0.5)

    # 進入到 forward 狀態時 
    def on_enter_forward(self):
      
      # 建立 Twist 訊息, 並給予向前速度 0.2, 持續0.5秒 
      msg = Twist()
      msg.linear.y = 0.2
      t1.controler.move(msg, 0.5)

    # 進入到 addSp 狀態時 
    def on_enter_addSp(self):
    
      # 建立 Twist 訊息, 並給予向前速度 0.4, 持續3秒
      msg = Twist()
      msg.linear.y = 0.4
      t1.controler.move(msg, 3)
      
      # 建立 Twist 訊息, 並給予向前速度 0.5, 持續3秒 
      msg = Twist()
      msg.linear.y = 0.5
      t1.controler.move(msg, 3)
      
# class MyModel
# 轉移鏈判斷模型
class MyModel(object):
    
    # 初始化 state 變數, state表示目前 過框有限狀態機 的狀態 
    def __init__(self, state):
        self.state = state
    
    # define run function
    # 執行轉移鏈判斷
    def run(self, fsm):
        
        # 開始執行控制迴圈, 終止條件為 roscore停止時, 或是被break
        while not rospy.is_shutdown():
        
            # 在視窗上顯示目前狀態
            print(self.state)
            
            # 當 目前有限狀態機狀態為 hover
            if fsm.hover.is_active:
                
                # 檢查 target_x 與 target_y 是否有收到訂閱值, 沒有收到的情況下皆為 -1, 因此狀態要轉移至 hover, 透過 wait4data 完成轉移
                if t1.state.target_x == -1 and t1.state.target_y == -1:
                    fsm.wait4data()
                # 當 target_x 與 target_y 不為 -1, 表示有收到訂閱值
                # 計算 target_x 與 中心點x 480 之間的差 dx 
                # 計算 target_y 與 中心點y 200 之間的差 dy 
                # 判斷可以向前或是需要校正
                else:
                  dx = t1.state.target_x - 480
                  dy = t1.state.target_y - 200
                  
                  # 可以向前, 轉移至 forward 狀態, 透過 start2forawrd 完成轉移
                  if abs(dx) < 30 and abs(dy) < 30:
                    fsm.start2forawrd()
                  
                  # 需要校正, 轉移至 correction 狀態, 透過 start2correct 完成轉移  
                  else:
                    fsm.start2correct()
            # 當 目前有限狀態機狀態為 correction
            elif fsm.correction.is_active:
                
                # 當canPass 為 1, 表示影像程式認為mask 面積已達到門檻值, 故此時轉移至加速過框的狀態 addSp, 透過 need2addSp 完成轉移
                if t1.state.canPass == 1:
                    fsm.need2addSp()
                #當 canPass 不為 1, 表示還需要向前或是校正
                else:
                    # 計算 target_x 與 中心點x 480 之間的差 dx 
                    # 計算 target_y 與 中心點y 200 之間的差 dy 
                    dx = t1.state.target_x - 480
                    dy = t1.state.target_y - 200
                    
                    # 可以向前, 轉移至 forward 狀態, 透過 need2forawrd 完成轉移
                    if abs(dx) < 30 and abs(dy) < 30:
                      fsm.need2forawrd()
                    
                    # 需要校正, 轉移至 correction 狀態, 透過 need2correct 完成轉移  
                    else:
                      fsm.need2correct()
            
            # 當 目前有限狀態機狀態為 forward
            elif fsm.forward.is_active:
                
                # 當canPass 為 1, 表示影像程式認為mask 面積已達到門檻值, 故此時轉移至加速過框的狀態 addSp, 透過 need2addSp 完成轉移
                if t1.state.canPass == 1:
                    fsm.need2addSp()
                 #當 canPass 不為 1, 表示還需要向前或是校正
                else:
                    # 計算 target_x 與 中心點x 480 之間的差 dx 
                    # 計算 target_y 與 中心點y 200 之間的差 dy  
                    dx = t1.state.target_x - 480
                    dy = t1.state.target_y - 200
                    
                    # 可以向前, 轉移至 forward 狀態, 透過 need2forawrd 完成轉移
                    if abs(dx) < 30 and abs(dy) < 30:
                      fsm.need2forawrd()
                    
                    # 需要校正, 轉移至 correction 狀態, 透過 need2correct 完成轉移  
                    else:
                      fsm.need2correct()
            # 當 目前有限狀態機狀態為 addSp, 表示可以中斷轉移
            elif fsm.addSp.is_active:
                break

# main function
# 負責執行一連串的指令, 如 起飛 開始過框之有限狀態機 降落
def main():
  
  while t1.state.is_flying == False:  # 當is_flying為False, 表示無人機還沒有成功起飛, 故透過while 迴圈不斷執行起飛, 直到is_flying為True, 表示無人機起飛成功
    t1.controler.takeoff()
  
  while t1.state.fly_mode != 6: # 當fly_mode != 6, 表示無人機正在執行特殊行為, 如 起飛 降落 翻滾等等, 因此透過while迴圈確認特殊行為是否結束
    print("wait...")
  
  # 建立轉移鏈判斷模型以及有限狀態機
  obj = MyModel(state='hover')
  fsm = sMachine(obj)
  
  # 執行
  obj.run(fsm)
  
  while t1.state.fly_mode != 6: # 當fly_mode != 6, 表示無人機正在執行特殊行為, 如 起飛 降落 翻滾等等, 因此透過while迴圈確認特殊行為是否結束
    print("wait...") 
    
  while t1.state.is_flying == True:  # 當is_flying為True, 表示無人機還正在飛行, 故透過while 迴圈不斷執行降落, 直到is_flying為False, 表示無人機完成降落
    t1.controler.land() 

# main
if __name__ == '__main__':

    # 告訴ros此程式為node, node名稱為 'h264_pub', anonymous為True,表示ros會幫此node加一段亂碼進行匿名
    rospy.init_node('h264_pub', anonymous=True)
    
    #執行 範例function main
    main()


