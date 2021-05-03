# 제 4회 국민대 자율주행 경진대회

## 기본 설치   
   ### 1. [VMware Worksation 설치(가상머신 설치)](#Vmware-Workstation-설치)
   ### 2. [Ubuntu 18.04 설치](#Ubuntu-18.04-설치)
   ### 3. [ROS Melodic 설치](#ROS-Melodic-설치)      

<br><br>   

# VMware Worksation 설치
VMware Workstation은 Windows나 Linux 운영체제에서 다른 운영체제를 설치하고 실행시키고 싶을 때 가상환경을 만들어 실행시켜주는 프로그램. Mac 운영체제는 Worksation이 아닌 Fusion을 설치
<a href="https://www.vmware.com/kr/products/workstation-player/workstation-player-evaluation.html">(공식 홈페이지)</a>   
|다운로드|Windows|Linux|Mac|
|---|:---:|:---:|:---:|
||<a href="https://www.vmware.com/go/getplayer-win">다운로드</a>|<a href="https://www.vmware.com/go/getplayer-linux">다운로드</a>|<a href="https://www.vmware.com/kr/products/fusion/fusion-evaluation.html">다운로드</a>|    

### 1. 설치 완료 후 `Create a New Virtual Machine` 선택   
<img src="https://ommant.com/imgs/KMU_Automobile/001.png"></img>   

### 2. 다운받은 iso 파일을 선택   
<img src="https://ommant.com/imgs/KMU_Automobile/002.png"></img>   

### 3. Full name과 User name 및 암호를 설정
크게 중요하지 않은 name이니 암호만 잘 기억하면 됨   
<img src="https://ommant.com/imgs/KMU_Automobile/003.png"></img>   

### 4. 가상 머신 이름을 지정 KMU 혹은 KMU_Automobile로 지정   
<img src="https://ommant.com/imgs/KMU_Automobile/004.png"></img>   

### 5. 가상 머신이 사용할 용량을 지정   
5GB도 쓰지 않으니 권장 사항에 맞춰 지정
<img src="https://ommant.com/imgs/KMU_Automobile/005.png"></img>   

### 6. 가상 머신의 하드웨어를 설정
성능에 영향을 미치는 Processor 혹은 Memory를 적당히 지정   
최대로 지정해도 큰 문제는 없음   
<img src="https://ommant.com/imgs/KMU_Automobile/006.png"></img>   
<br><br>

# Ubuntu 18.04 설치
1. Ubuntu 18.04 iso 파일 다운받기(<a href="https://cloud.ommant.com/index.php/s/nzGaT2TD5qSmNtb">다운</a>)
2. VMware에 Ubuntu 18.04 설치하기   
3. 설치 후 기본작업 시작하기   
   3-1. 패키지 설치 주소를 카카오로 설정   
```
$ sudo vi /etc/apt/source.list
:%s/us.archive.ubuntu.com/mirror.kakao.com
# 14줄이 대체되었다는 메세지가 뜸
:wq!
# 저장하고 종료
$ sudo apt update
$ sudo apt upgrade -y
$ sudo apt install vim
```
<br><br>   

# ROS Melodic 설치    
1. Ubuntu install of ROS Melodic   
    1-1. sources.list 설정하기
    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc)     main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
    1-2. 키 설정하기
    ```bash
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

    =>
    gpg: key F42ED6FBAB17C654: "Open Robotics <info@osrfoundation.org>" not changed
    gpg: Total number processed: 1
    gpg:              imported: 1
    ```
    1-3. 설치하기
    ```bash
    sudo apt update
    ```
    ```
    sudo apt install ros-melodic-desktop-full -y
    ```
    1-4. 빌드 의존성 패키지 설치하기   
    ```bash
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
    ```
    1. rosdep 초기화하기
    ```bash
    sudo apt install python-rosdep

    sudo rosdep init
    rosdep update
    ```
2. ROS 워크스페이스 생성   
설치 후 ROS 작업을 할 작업 공간을 생성
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make

    =>
    ~~~~~~~~
    ####
    #### Running command: "make -j8 -l8" in "/home/####/catkin_ws/build"
    ####
    ```
3. ROS 작업환경 설정   
    * ROS 작업에 필요한 환경변수 설정   
    * 홈 디렉토리에 있는 bash 소스 파일 수정
        ```
        cd
        sudo vi ~/.bashrc
        ```
        G 입력 후 아래 내용 입력   
        ```bash
        ...
        alias cm='cd ~/catkin_ws && catkin_make'
        source /opt/ros/melodic/setup.bash
        source ~/catkin_ws/devel/setup.bash
        export ROS_MASTER_URI=http://localhost:11311
        export ROS_HOSTNAME=localhost
        ```
        저장 후 `source .bashrc` 입력
4. 시뮬레이터 의존 패키지 설치   
    * 과제 수행에 필요한 2D 시뮬레이터를 사용하려면 먼저 아래 의존 패키지들의 설치가 필요
    * 설치할 의존 패키지들 (아래 순서대로 설치)   
        * ar-track-alvar v0.7.1     - Alvar AR Tag 전용 ROS 패키지
        * pygame v1.9.6             - 파이썬 게임 제작 라이브러리
        * pillow v6.2.2             - 파이썬 이미지 처리 라이브러리   

    1. ar-track-alvar for ROS Melodic   
        ```bash
        sudo apt update   
        sudo apt install ros-melodic-ar-track-alvar
        ```
    2. Python pip 설치
        * pygame 과 pillow 를 설치하려면 pip(python package index) 가 필요
        * pip 는 파이 썬 으로 작성된 패키지 소프트웨어를 설치 · 관리하는 패키지 관리 시스템
        * pip 는 ubuntu 18.04 의 기본 패키지가 아니므로 직접 설치한다
        * `sudo apt install python-pip`
    3. Python Pygame 설치
        * 설치 시 에러 발생하기 때문에 아래의 의존성 라이브러리 설치
            ```bash
            sudo apt-get install python-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libsmpeg-dev python-numpy subversion libportmidi-dev ffmpeg libswscale-dev libavformat-dev libavcodec-dev libfreetype6-dev
            ```
        * `pip install pygame==1.9.6`
    4. Pillow 설치
        * `pip install pillow=6.2.2`