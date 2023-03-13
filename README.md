# erp42_sim_ws

1.터미널에 다음과 같이 입력하여 코드를 다운로드 받습니다.

git clone https://github.com/WeGo-Robotics/erp42_sim_ws.git 

2.다운로드 받은 erp42_sim_ws 폴더로 이동합니다.

cd erp42_sim_ws

3.epr42_sim_ws 폴더에서 catkin_make를 실행합니다.

catkin_make

4.사용하시는 shell에 맞게 아래 두 명령어 중 선택하여 입력하여,

  패키지를 등록합니다.(bash shell - 1번 명령어, zsh shell- 2번 명령어)
  
source devel/setup.bash

source devel/setup.zsh

5.터미널에 다음과 같이 입력하여, ros master를 실행합니다.

roscore

6.roscore가 실행되었다면, 터미널에 다음과 같이 입력하여 파일을 실행해 봅니다.

ex)

rosrun opencv_edu 1.\ grayscale.py

7.실행이 되지 않는다면, 파일의 권한이 부여되지 않은 것임으로, 해당 scripts 폴더로 이동하여, 권한을 부여합니다.

cd <<PACKAGE_NAME>>/script

sudo chmod 777 *

