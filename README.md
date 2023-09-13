# RDLVY
[![GitHub license](https://img.shields.io/github/license/Naereen/StrapDown.js.svg)](https://github.com/Naereen/StrapDown.js/blob/master/LICENSE)

We modified the robotic-warehouse environment [RWARE](https://github.com/semitable/robotic-warehouse)[1] to create a gird based robotic-delivery environment.

Below is a rendering of 4 agents with random policy in small (9x14) RDLVY envrionment.

<p align="center">
 <img width="300px" src="rdlvy_docs/RDLVY_simple.gif" align="center" alt="RDLVY illustration" />
</p>

Our research will be done on a map modeled after Sangam-dong, Seoul, Repulic of Korea.

<p align="center">
 <img width="600px" src="rdlvy_docs/Sangam-dong.png" align="center" alt="RDLVY illustration" />
</p>

<h1>Table of Contents</h1>

- [RDLVY](#rdlvy)
  - [Getting started](#getting-started)
  - [WorkFlow](#workflow)
  - [Env Description](#env-description)
    - [Package](#package)
    - [HighWay Rules](#highway-rules)
    - [Layout Rules](#layout-rules)
  - [Observations](#observations)
    - [Image observations description](#image-observations-description)
    - [\[WIP\] Dictionary observations description](#wip-dictionary-observations-description)
  - [Reward](#reward)
- [Reference](#reference)


## Getting started
```shell
cd RDLVY
export PYTHONPATH=$PYTHONPATH:.
python rware/warehouse.py
```

## WorkFlow
Git Workflow: Fork & Pull Request
Formatter: Use black

## Env Description
시작 지점 (물류창고) & package
- 다수의 시작 지점에서 n개의 package가 생성
- 한 개의 시작 지점에 여러개의 package 존재 가능
- 각 pacakge는 모적지가 지정되어 있지 않음

목적지 (배달지)
- n개의 목적지(배달지) 생성
- 목적지에 package를 가진 agent가 도착하면 리워드 생성
  - 해당 목적지는 없어지며 새로운 목적지 생성

Agent
- Agent들이 시작지점에서 package를 load 후 목적지에 배달
- Agent는 여러개의 package를 담을 수 있음

이동 규칙
- 한국의 도로 운행법을 따름
- 직진 및 우회전은 자유롭게 가능
- 유턴 및 좌회전은 교차로(intersection)에서 가능
- 중앙선 침범은 원체 불가능
- 중앙선 방향으로 agent direction 변경 불가

### Package
1. start에서 여러개의 package를 생성
- Package를 담는 request_queue 생성
    - start_candidate 중 하나의 위치에서 생성된 pacakge가 request_queue에 추가됨
    - 같은 위치에 여러개의 package가 생성되어 queue에 추가 가능
    - request_queue는 shelf 사용 시와 달리 start 점에서 package가 agent에 load되면 해당 package가 queue에서 빠짐
    - package가 goal에 도착 시 request_queue에 새로운 package 추가
- starts_with_package_grid 추가
    - 2D girdmap으로 각 cell은 시작 점에 쌓인 package 수를 나타냄
    - obs에 start 점에서 기다리는 pacakge의 위치를 알려주기 위해 생성
2. goal 구현
- goal은 goal_candidates 중에 무작위로 생성
- goal에 agent가 도착하면 package를 가지고 있을 시 자동으로 package 하나를 unload
    - 새로운 goal 생성되고 배달 완료된 goal 없어짐

### HighWay Rules
1. Highways 
- Highway는 agent가 다닐 수 있는 모든 block을 지칭함 (goal 및 start 포함)
    - Highway block은 block마다 해당 도로가 어떤 방향으로 향하는 도로인지인 정보를 가지고 있음(highways_info)
    - Highways info를 따라 agent가 진행할 수 있는 방향이 정해짐
    - 갈 수 있는 진행 방향이 나올 때까지 해당 위치에서 방향 변경(역주행 및 중앙선 침범 금지)

### Layout Rules
Layout의 변경을 원하는 경우 아래의 규칙을 따라 작성해야만 도로 주행 로직이 정상적으로 작동함
1. Layout의 모든 point는 'r', 'l', 'u', 'd', '.', 'x', 's', 'g' 중 하나여야 함
2. 'x'의 경우 highway가 아닌 장애물을 나타냄
3. 'r', 'l', 'u', 'd'의 경우, intersection을 제외한 highway를 구성하고 해당 point에서 highway info를 결정함
- 사거리 및 삼거리
    - 사거리 및 삼거리의 경우 intersection layout('.')을 !!사용하지 않고!! 구성해야 함.

          4 way highway example
          ...
          x x x x d u x x x x
          x x x x d u x x x x
          x x x x d u x x x x
          l l l l d l l l l l
          r r r r r u r r r r
          x x x x d u x x x x
          x x x x d u x x x x
          ...

          3 way highway example
          ...
          x x x x d u x x x x
          x x x x d u x x x x
          x x x x d u x x x x
          x x x x d u x x x x
          l l l l d l l l l l
          r r r r r r r r r r
          x x x x x x x x x x


4. '.'는 intersection을 나타내는 layout으로, 단방향 코너 (!!사거리, 삼거리 미포함!!) 및 layout의 가장자리까지 도로가 연결되는 경우에서만 사용해야 함
- Intersection 
    - Intersection point에서 highway info는 ALL
    - Intersection에는 두 가지 type이 있음
        - Intersection 1

          the () point: inner corner point of corner street.
          Intersection 1에서는 1방향으로만 진행 가능

              x x x x d u x x x x
              l l l l(.)u x x x x
              r r r r r . x x x x
              x x x x x x x x x x
    
        - Intersection 3

          the () point: outer corner point of corner street.
          Intersection 3에서는 3방향 탐색 및 전진 가능

              x x x x d u x x x x
              l l l l . u x x x x
              r r r r r(.)x x x x
              x x x x x x x x x x

    - Layout 가장자리에서 사용하는 경우, Uturn을 위한 포인트로, 가장자리에 진입하는 point에만 사용해야 함

          Edge point example
          x x x x d(.)x x x x
          x x x x d u x x x x
          x x x x d u x x x x
          ...

5. 's'는 start point를 나타냄.
- start point는 물건을 싣는 곳으로, rendering 상에서는 빨간색으로 표시되고, starting point에 적재된 package가 없을 경우 옅은 분홍색

6. 'g'는 goal point candidate를 나타냄.
- Layout에서 g로 표시된 곳 중 파라미터의 goal포인트 수 만큼 goal을 랜덤하게 선택하여 초기화. Goal point로 선정된 곳에 패키지가 배달되면 해당 포인트는 goal point에서 제거되며 새로운 goal point가 goal point candiate 중에 랜덤하게 선택되어 생성됨.
- Layout에서 'g'는 상당히 많아도 됨. 길을 따라 모든 곳에 생성 가능.


## Observations
### Image observations description
- SEHLVES: represents shelves(=obstacle)
- REQUESTS: represents requested 'packages'
- AGENTS
- AGENT_DIRECTION
- AGENT_LOAD: if agents are carrying packages(1) or not(0)
- GOALS: represents requested goals
- ACCESSIBLE: 0 if cell is occupied by other agent

- Env 생성 시 observation_type=ObsevationType.IMAGE, use_full_obs=True option으로 전체 obs에 접근 가능
- image_observation_layers=[

    ImageLayer.SHELVES,

    ImageLayer.REQUESTS,

    ImageLayer.AGENTS,

    ImageLayer.AGENT_DIRECTION,

    ImageLayer.AGENT_LOAD,

    ImageLayer.GOALS,

    ImageLayer.ACCESSIBLE

  ]

  로 원하는 layer 중 선택 가능


### [WIP] Dictionary observations description
obs["self"]
- location
- carrying_shelf -> represents number of carrying 'package'
- direction
- on_highway

obs["sensors"][i]
- has_agents
- direction
- local_message
- has_shelf -> represents shelf(=obstacle)
- shelf_requested -> represents if there is 'package' requested
- goal_requested -> represents if there is 'goal' requested


## Reward
- Env 생성 시 reward_type= 파라미터로 조절 가능
- RewardType.GLOBAL: delivery 완료 시 모든 agent가 +1의 reward를 받음
- RewardType.INDIVIDUAL: delivery 완료 시 해당 agent만 +1의 reward를 받음
- RewardType.TWO_STAGE: 해당 agent만 package를 load할 때 +0.5, deliver를 완료할 때 +0.5의 reward를 받음

# Reference
[1] Filippos, C., Schäfer, L., & Albrecht, S. (2020). Shared Experience Actor-Critic for Multi-Agent Reinforcement Learning [Conference paper]. 33, 10707–10717. https://arxiv.org/abs/2006.07169
