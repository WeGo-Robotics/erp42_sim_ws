# erp42_sim_ws

1.터미널에 다음과 같이 입력하여 코드를 다운로드 받습니다.

git clone https://github.com/WeGo-Robotics/erp42_sim_ws.git 

2.다운로드 받은 erp42_sim_ws 폴더로 이동합니다.

cd erp42_sim_ws

3. 터미널에 다음과 같이 입력하여, 원클릭 셋팅 파일의 모든 권한을 열어줍니다.
sudo chmod 777 permission.sh

4. 터미널에 다음과 같이 입력하여 원클릭 셋팅 파일을 실행합니다. 

※원클릭 셋팅 파일(permission.sh)은 빌드(catkin_make) 및 사용하시는 쉘(bash or zsh)에 맞춰서
쉘의 환경설정 파일(~/.bashrc or ~/.zshrc)에 현재 워크스페이스($Current_path)의 경로를
설정(source devel/setup.bash or source devel/setup.zsh)합니다.

./permission.sh

3.터미널에 다음과 같이 입력하여, ros master를 실행합니다.

roscore

4.roscore가 실행되었다면, 터미널에 다음과 같이 입력하여 파일을 실행해 봅니다.

ex)

rosrun opencv_edu 1.grayscale.py

5.rostopic list를 입력하여, 코드가 실행되어 있는지 확인합니다.

rostopic list

![image](https://user-images.githubusercontent.com/113410253/224614156-faedebe0-9b34-435a-89e8-41a4ea2d56cd.png)

6.rqt_graph를 입력하여, 노드의 이름과 토픽을 확인합니다.

rqt_graph

![image](https://user-images.githubusercontent.com/113410253/224614311-04562d68-b6d2-4a25-b3a3-179350ccbe5d.png)


7. rqt_image_view를 실행하여, 결과를 확인합니다.

![image](https://user-images.githubusercontent.com/113410253/224614482-9f10f471-dbf3-494b-a83f-1e4ced44baf4.png)

