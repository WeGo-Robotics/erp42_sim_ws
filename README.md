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


5.다운로드 받은 파일의 권한이 부여되지 않았으므로, 해당 scripts 폴더로 이동하여, 권한을 부여합니다.

cd <<PACKAGE_NAME>>/script

ex) 

roscd opencv_edu

cd scripts

sudo chmod 777 *

6.터미널에 다음과 같이 입력하여, ros master를 실행합니다.

roscore

7.roscore가 실행되었다면, 터미널에 다음과 같이 입력하여 파일을 실행해 봅니다.

ex)

rosrun opencv_edu 1.grayscale.py

8.rostopic list를 입력하여, 코드가 실행되어 있는지 확인합니다.

rostopic list

![image](https://user-images.githubusercontent.com/113410253/224614156-faedebe0-9b34-435a-89e8-41a4ea2d56cd.png)

9.rqt_graph를 입력하여, 노드의 이름과 토픽을 확인합니다.

rqt_graph

![image](https://user-images.githubusercontent.com/113410253/224614311-04562d68-b6d2-4a25-b3a3-179350ccbe5d.png)


10. rqt_image_view를 실행하여, 결과를 확인합니다.

![image](https://user-images.githubusercontent.com/113410253/224614420-ac55471f-14bb-4088-bcc2-1516f25b6b27.png)

