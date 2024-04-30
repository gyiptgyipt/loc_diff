### rom2109_description

#### Simulation  ပြုလုပ်ရန်
ပုံမှန်အားဖြင့် gazebo ထဲသို့ spawn လုပ်ရာတွင် topic, database, file နည်းလမ်းများဖြင့် လုပ်နိုင်သော်လည်း topic နည်းလမ်းသည်  robot_state_pubisher  မှ လာတာမို့လို့ urdf, xacro, sdf တွေမှာပြသနာရှိနိုင်ပါတယ်။ ဒါကြောင့်  database, file  နည်းလမ်းများကို သုံးထားပါတယ်။

###### နည်းလမ်း (၁) ပုံမှန်  gazebo controller တွေနဲ့ simulation ပြုလုပ်လိုရင်
```
ros2 launch rom2109_gazebo rom2109_sim_gz_control.launch.py use_sim_time:=true
```
###### နည်းလမ်း (၂) gazebo ros2 control  နဲ့ simulation ပြုလုပ်လိုရင်
```
ros2 launch rom2109_gazebo rom2109_sim_ros2_control.launch.py
```
###### နည်းလမ်း (၃) Gazebo မှ တဆင့် simulation
```
ros2 launch gazebo_ros gazebo.launch.py
# insert ကိုနှိပ်ပြီး rom2109_tall or rom2109_tall_ros or rom2109_bot တခုခုထည့်ပါ။
# rom2109_tall_ros သုံးပြီး gazebo ros2 controller manager ကို လိုချင်ရင် 
# rom2109_description package ထဲက description_ros2_control.launch.py ကို 
# launch လုပ်ဖို့လိုအပ်ပါတယ်။
```


<a href="https://github.com/ROM-robotics/rom2109_simulation"> မူလစာမျက်နှာ </a>