# 스마트 글러브 수어 번역 시스템 (Smart Glove)

수어(숫자 0~9)의 움직임을 인식하여 텍스트로 번역하는 STM32 기반 스마트 글러브 시스템.

## 프로젝트 구조
- `Core/`, `Drivers/` : STM32 펌웨어 소스코드 (STM32CubeIDE)
- `project.ioc` : STM32CubeMX 핀 및 페리페럴 설정 파일
- `마프실_13조_프로젝트보고서_final.pdf` : 프로젝트 구현 내용 및 결과 보고서

## 하드웨어 및 개발 환경
- MCU : STM32F103RBT6
- Firmware : STM32CubeIDE, C (HAL API)

## 빌드 및 실행
1. STM32CubeIDE에서 해당 프로젝트 임포트.
2. 빌드(Build) 후 ST-LINK를 통해 보드에 바이너리 다운로드.
