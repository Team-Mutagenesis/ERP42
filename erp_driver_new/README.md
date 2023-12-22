# ERP42 driver

<br/>

## 1.erp_driver_new 폴더로 이동.

```
cd erp42_ws
```

<br/>

## 2. 터미널에 다음과 같이 입력하여, 원클릭 셋팅 파일의 권한을 열어줍니다.
```
sudo chmod 777 permission.bash
```

<br/>

## 3. 터미널에 다음(password for $USER:)과 같이 출력되면 패스워드를 입력합니다.

<br/>


## 4. 터미널에 다음과 같이 입력하여 원클릭 셋팅 파일을 실행합니다.
```
./permission.bash
```
※원클릭 셋팅 파일(permission.sh)은 빌드(catkin_make) 및 사용하시는 쉘(bash or zsh)에 맞춰서 쉘의 환경설정 파일(~/.bashrc or ~/.zshrc)에 현재 워크스페이스($Current_path)의 경로를 설정(source devel/setup.bash or source devel/setup.zsh)합니다.

<br/>

## 5. 터미널 종료 후 다시 실행하여 명령어를 다음과 같이 입력하여 실행합니다.
```
roslaunch erp_driver erp42_base.launch
```

<br/>

## 6. rostopic list를 입력하여, 코드가 실행되어 있는지 확인합니다.
```
rostopic list
```
