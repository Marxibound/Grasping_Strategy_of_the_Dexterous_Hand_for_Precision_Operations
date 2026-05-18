from inspire_hand import *
import time
# set_clear_error()
setpos(0,0,0,0,0,0)
setpower(1000,1000,1000,1000,1000,1000)
setspeed(100,100,100,100,300,300)
time.sleep(1) 
setpos(0,0,0,0,600,1000)
time.sleep(1)
print(get_actpos())
setpos(0,0,0,400,600,1700)
time.sleep(1) 
# print(get_actpos())
# setpos(0,0,0,400,600,1700)
# time.sleep(1)

print("按空格键将机械手位置设置为零，按q键退出程序")
try:
    while True:
        key = input()
        if key == " ":
            print("重置机械手位置...")
            setpos(0,0,0,0,0,0)
            # setangle(1000,1000,1000,1000,1000,1000)
            time.sleep(1)
            print(get_actpos())
        elif key == "q":
            print("退出程序")
            break
except KeyboardInterrupt:
    print("程序被中断")
finally:
    ser.close()