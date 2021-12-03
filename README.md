# Autonomous-Driving-Simulator
**자율주행 시뮬레이터 개발**  


## Description
본 프로젝트에서는 세 가지 모듈을 이용하여 자율주행 시뮬레이션을 구현하였다. 각각 입출력을 담당하는 GUI 모듈, HDmap을 이용하여 조향각을 계산하는 HDmap 모듈, 차선 인식 결과를 바탕으로 조향각을 계산하는 LaneDetection 모듈이다. GPS가 FLOATING 상태에서 dead reckoning을 통한 결과와 차선 인식 결과, map을 퓨징하여 위치 추정 오차를 줄이기 위한 방법론에 대해 개발을 진행하였다.   

## Core Features
### 차선 인식 ### 
+ Hough Transform  
ROI로 지정한 영역에 대해 허프 변환 진행
+ Filtering  
탐색한 선분에 대해 차선으로 적합한 기울기 범위 내의 차선 filtering 진행
+ Fit line  
검출한 선분을 연결하여 하나의 직선으로 변환, 차선으로 인식
+ 차선 업데이트  
이전 프레임에서 찾은 차선과 새로 찾은 차선의 유사도를 이용하여 차선 업데이트  

![차선인식 사진](https://user-images.githubusercontent.com/71866756/144534105-263b7156-3ff3-453a-943b-3106f33f8ba5.png)

### HDmap을 이용한 경로 탐색 ###
+ Localization  
GPS가 FIXED 상태에서 map과 비교를 통해 현재 위치 추정  
GPS가 FLOATING 상태에서 간략한 Dead Reckoning을 통한 위치 추정  

+ 경로 생성  
최단 경로 탐색 알고리즘(다익스트라)을 통한 경로 생성 

![경로](https://user-images.githubusercontent.com/71866756/144534714-c0761ab7-ba46-4433-b134-e8f40b461f3a.png)

### 입/출력 ###
+ User Input  
GPS on/off 상태 조절, 카메라가 설치된 위치, look ahead distance 등 사용자가 임의로 parameter setting  

+ Output  
HDmap, LaneDetection모듈을 통해 계산한 조향각 및 주행 화면, cross track error 등 출력 

![GUI](https://user-images.githubusercontent.com/71866756/144534337-b8cc936b-04bf-4c19-8443-d0c12ba61040.png)

## Structure ##  
![rqt graph](https://user-images.githubusercontent.com/71866756/144534364-daeb1207-cf0d-42d7-8d18-a2fd45375531.png)

## Demonstration video ##
## Env
+ ROS melodic
+ Ubuntu 18.04
+ OpenCV
+ QT designer  
