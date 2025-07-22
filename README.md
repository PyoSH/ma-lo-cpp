# MApping-Localization with CPP (MA-LO-CPP) for B1 quadruped
## Intro
임형태 박사님의 [카이스트 수업자료](https://github.com/LimHyungTae/mcl_2d_lidar_ros) 를 기반으로 만들었으나, ROS 구현 과정에서 많은 부분을 변경.  
2D LiDAR 대신 두 개의 리얼센스 스테레오 카메라-RS 435, RS 455 를 사용.  

''' bash
@article{ART003222771,
author={구본학 and 이승연 and 표승현 and 전봉환 and 박대길},
title={다관절 해저보행로봇의 자율운용기술 개발을 위한 선행 연구},
journal={대한기계학회논문집 A},
issn={1226-4873},
year={2025},
volume={49},
number={7},
pages={503-517}
}
'''
## Mapping 

## Localization (MCL implement)
<p align='center'>
  <img src="https://github.com/user-attachments/assets/08eb3034-f336-49a3-a8be-b0bd5c90dc72">
</p>


### Real-Time Implementation
기존) deque 자료구조로 발행된 시간을 비교 후 사용하는 방식  
본 코드) 타이머 인터럽트로 predict, update, publish 수행
