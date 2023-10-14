CANopenNode
===========

CANopenNode는 오픈소스 CANopen protocol stack이다.

CANopen은 국제표준(EN 50325-4) ([CiA301](http://can-cia.org/standardization/technical-documents))으로 CAN 위에 구축된 임베디드 제어 시스템을 위한 상위수준 프로토콜이다. CANopen에 대한 자세한 내용은 http://www.can-cia.org/를 참조하십시오.

CANopenNode는 ANSI C로 작성되었으며 객체지향 방식으로 작성되었다. 다양한 마이크로컨트롤러에서 독립적인 응용프로그램 형식 혹은 RTOS와 함께 실행된다.
Variables (communication, device, custom)은 CANopen Object Dictionary에 수집되며 : C 코드와 CANopen 네트워크로 접근이 가능하다.

CANopenNode 홈페이지는 https://github.com/CANopenNode/CANopenNode

새로운 Object Dictionary 구현을 포함한 CANopenNode의 버전 4는 2021년 7월에 출시되었다. 이전 버전은 `git checkout` branches `v1.3-master` 또는 `v2.0-master`를 사용하십시오.


Characteristics
---------------
### CANopen
 - [Object Dictionary](https://www.can-cia.org/can-knowledge/canopen/device-architecture/) 변수에 대한 명확하고 유연한 구조를 제공. 변수에 직접 접근하거나 읽기/쓰기 함수를 통해 직접 접근할 수 있다.
 - [NMT](https://www.can-cia.org/can-knowledge/canopen/network-management/) start, stop, reset device 시키기 위한 slave. 간단한 NMT master.
 - [Heartbeat](https://www.can-cia.org/can-knowledge/canopen/error-control-protocols/) CANopen 장치 모니터링을 위한 producer/consumer error 제어
 - [PDO](https://www.can-cia.org/can-knowledge/canopen/pdo-protocol/) for broadcasting process data with high priority and no protocol overhead. Variables from Object Dictionary can be dynamically mapped to the TPDO, which is then transmitted according to communication rules and received as RPDO by another device.
 - [SDO](https://www.can-cia.org/can-knowledge/canopen/sdo-protocol/) server는 CANopen 장치 내의 모든 Object Dictionary 변수에 대한 빠른, 세그먼트화된, 블록 전송 접근을 가능하게 한다.
 - [SDO](https://www.can-cia.org/can-knowledge/canopen/sdo-protocol/) client는 네트워크 내의 모든 CANopen 장치의 Object Dictionary 변수에 접근할 수 있다.
 - [Emergency](https://www.can-cia.org/can-knowledge/canopen/special-function-protocols/) message producer/consumer.
 - [Sync](https://www.can-cia.org/can-knowledge/canopen/special-function-protocols/) producer/consumer는 PDO 객체의 네트워크 동기화 전송을 가능하게 한다.
 - [Time-stamp](https://www.can-cia.org/can-knowledge/canopen/special-function-protocols/) producer/consumer enables date and time synchronization in millisecond resolution.
 - [LSS](https://www.can-cia.org/can-knowledge/canopen/cia305/) CANopen node-id와 bitrate 설정, master과 slave, LSS fastscan.
 - [CANopen gateway](https://www.can-cia.org/can-knowledge/canopen/cia309/), NMT master, LSS master와 SDO client에 대한 CiA309-3 Ascii command interface
 - CANopen Safety, EN 50325-5, CiA304, "PDO like" communication in safety-relevant networks
 - [CANopen Conformance Test Tool](https://www.can-cia.org/services/test-center/conformance-test-tool/) passed.

### Other
 - [Suitable for 16-bit microcontrollers and above](#device-support)
 - [Multithreaded, real-time](#canopenNode-flowchart)
 - [Object Dictionary editor](#object-dictionary-editor)
 - Object Dictionary 변수 혹은 다른 변수에 대해서 Non-volatile 저장소. 자동으로 혹은 표준 CANopen 명령, 설정으로 제어가능
 - [Power saving possible](#power-saving)
 - [Bootloader possible](https://github.com/CANopenNode/CANopenNode/issues/111) (firmware 업데이트용)


관련 프로젝트
----------------
 - [CANopenNode](https://github.com/CANopenNode/CANopenNode) (this project): CANopen protocol stack, base for CANopen device. It contains no device specific code (drivers), which must be added separately for each target system. An example shows the basic principles, compiles on any system, but does not connect to any CAN hardware.
 - [CANopenDemo](https://github.com/CANopenNode/CANopenDemo): Demo device with CANopenNode and different target systems, tutorial and testing tools.
 - [CANopenNode.github.io](https://github.com/CANopenNode/CANopenNode.github.io): Html documentation, compiled by doxygen, for CANopenDemo, CANopenNode and other devices, available also online: https://canopennode.github.io
 - [CANopenEditor](https://github.com/CANopenNode/CANopenEditor): Object Dictionary editor, external GUI tool for editing CANopen Object Dictionary for custom device. It generates C source code, electronic data sheet and documentation for the device. It is a fork from [libedssharp](https://github.com/robincornelius/libedssharp).
 - [CANopenLinux](https://github.com/CANopenNode/CANopenLinux): CANopenNode on Linux devices. It can be a basic CANopen device or more advanced with commander functionalities.
 - [CANopenSTM32](https://github.com/CANopenNode/CanOpenSTM32): CANopenNode on STM32 microcontrollers.
 - [CANopenPIC](https://github.com/CANopenNode/CANopenPIC): CANopenNode on PIC microcontrollers from Microchip. Works with 16-bit and 32 bit devices. Includes example for Arduino style [Max32](https://reference.digilentinc.com/reference/microprocessor/max32/start) board.
 - [doc/deviceSupport.md](doc/deviceSupport.md): List of other implementations of CANopenNode on different devices.


문서, 지원, 기여하기
----------------------------------------
모든 code에 대한 문서는 header 파일내에 기술되어 있다. 일부 추가 문서는 `doc` 디렉토리에 있다.

html 문서를 생성하려면 프로젝트 내에 base 디렉토리내에서 [doxygen](http://www.doxygen.nl/)을 실행한다. : `sudo apt install doxygen graphviz pdf2svg; doxygen > /dev/null`

Complete generated documentation is also available online: https://canopennode.github.io

Tutorial, demo device and tests are available in [CANopenDemo](https://github.com/CANopenNode/CANopenDemo) repository.

Report issues on https://github.com/CANopenNode/CANopenNode/issues

Older discussion group is on Sourceforge: http://sourceforge.net/p/canopennode/discussion/387151/

Contributions are welcome. Best way to contribute your code is to fork a project, modify it and then send a pull request. Please follow the [Recommended C style and coding rules](https://github.com/MaJerle/c-code-style), like indentation of 4 spaces, etc. There is also a `codingStyle` file with example.


CANopenNode flowchart
---------------------
일반적인 CANopenNode 구현에 대한 Flowchart :
~~~
                            -----------------------
                           |     Program start     |
                            -----------------------
                                       |
                            -----------------------
                           |     CANopen init      |
                            -----------------------
                                       |
                            -----------------------
                           |     Start threads     |
                            -----------------------
                                 |     |     |
             --------------------      |      --------------------
            |                          |                          |
 ----------------------    ------------------------    -----------------------
| CAN receive thread   |  | Timer interval thread  |  | Mainline thread       |
|                      |  |                        |  |                       |
| - Fast response.     |  | - Realtime thread with |  | - Processing of time  |
| - Detect CAN ID.     |  |   constant interval,   |  |   consuming tasks     |
| - Partially process  |  |   typically 1ms.       |  |   in CANopen objects: |
|   messages and copy  |  | - Network synchronized |  |    - SDO server,      |
|   data to target     |  | - Copy inputs (RPDOs,  |  |    - Emergency,       |
|   CANopen objects.   |  |   HW) to Object Dict.  |  |    - Network state,   |
|                      |  | - May call application |  |    - Heartbeat.       |
|                      |  |   for some processing. |  |    - LSS slave        |
|                      |  | - Copy variables from  |  | - Gateway (optional): |
|                      |  |   Object Dictionary to |  |    - NMT master       |
|                      |  |   outputs (TPDOs, HW). |  |    - SDO client       |
|                      |  |                        |  |    - LSS master       |
|                      |  |                        |  | - May cyclically call |
|                      |  |                        |  |   application code.   |
 ----------------------    ------------------------    -----------------------
~~~

CANopenNode의 모든 코드는 non-blocking이다. 소스 파일의 코드는 객체로 수집된다. 코드의 일부는 enable/disable가 가능하다. 따라서 프로젝트에 필요한 파일과 코드 부분만 사용할 수 있다. 301/CO_config.h 파일에서 stack configuration을 확인하자.

효율적인 코드를 위해서 CANopenNode는 다른 thread에서 실행될 수 있다. 이는 마이크로컨트롤러에 적합하다. 모든 코드를 단일 thread에서 실행할 수도 있다. 이는 Linux 장치에서 가능하다. 코드는 필요할 때마다 OD 객체의 처리를 트리거하는 메커니즘을 포함한다.

CANopen 초기화 섹션에서 모든 CANopen 객체가 초기화된다. 실행시에 CANopen 객체가 주기적으로 처리된다.

CANopen.h와 CANopen.c 파일은 모든 CANopen 객체를 묶어놨다. 복잡해 보일 수 있지만, 유연성을 제공하며 대부분의 일반적인 CANopen 객체 설정에 적합하다. CANopen 객체는 전역 공간에서 정의될 수도 있고 동적으로 할당될 수도 있다. Object Dictionary는 기본(OD.h/.c files)으로 사용할 수 있지만, #CO_config_t 구조체를 사용하여 여러 Object Dictionary로 구성할 수도 있다. CANopen.h와 CANopen.c 파일은 CANopenNode 기반 장치의 커스텀 구현에 대한 참조일 수도 있다.

Object Dictionary는 모든 네트워크 접근 가능한 변수의 모음이며 유연하게 사용이 가능하다. OD 변수는 Object Dictionary에서 초기화 되거나 응용프로그램이 특정 OD 변수에 대한 읽기/쓰기 접근 함수를 지정할 수 있다. OD 변수의 그룹은 non-volatile 메모리에 저장될 수도 있고 명령 자동으로 저장될 수 있다.


File structure
--------------
 - **301/** - CANopen application layer and communication profile.
   - **CO_config.h** - Configuration macros for CANopenNode.
   - **CO_driver.h** - Interface between CAN hardware and CANopenNode.
   - **CO_ODinterface.h/.c** - CANopen Object Dictionary interface.
   - **CO_Emergency.h/.c** - CANopen Emergency protocol.
   - **CO_HBconsumer.h/.c** - CANopen Heartbeat consumer protocol.
   - **CO_NMT_Heartbeat.h/.c** - CANopen Network management and Heartbeat producer protocol.
   - **CO_PDO.h/.c** - CANopen Process Data Object protocol.
   - **CO_SDOclient.h/.c** - CANopen Service Data Object - client protocol (master functionality).
   - **CO_SDOserver.h/.c** - CANopen Service Data Object - server protocol.
   - **CO_SYNC.h/.c** - CANopen Synchronisation protocol (producer and consumer).
   - **CO_TIME.h/.c** - CANopen Time-stamp protocol.
   - **CO_fifo.h/.c** - Fifo buffer for SDO and gateway data transfer.
   - **crc16-ccitt.h/.c** - Calculation of CRC 16 CCITT polynomial.
 - **303/** - CANopen Recommendation
   - **CO_LEDs.h/.c** - CANopen LED Indicators
 - **304/** - CANopen Safety.
   - **CO_SRDO.h/.c** - CANopen Safety-relevant Data Object protocol.
   - **CO_GFC.h/.c** - CANopen Global Failsafe Command (producer and consumer).
 - **305/** - CANopen layer setting services (LSS) and protocols.
   - **CO_LSS.h** - CANopen Layer Setting Services protocol (common).
   - **CO_LSSmaster.h/.c** - CANopen Layer Setting Service - master protocol.
   - **CO_LSSslave.h/.c** - CANopen Layer Setting Service - slave protocol.
 - **309/** - CANopen access from other networks.
   - **CO_gateway_ascii.h/.c** - Ascii mapping: NMT master, LSS master, SDO client.
 - **storage/**
   - **CO_storage.h/.c** - CANopen data storage base object.
   - **CO_storageEeprom.h/.c** - CANopen data storage object for storing data into block device (eeprom).
   - **CO_eeprom.h** - Eeprom interface for use with CO_storageEeprom, functions are target system specific.
 - **extra/**
   - **CO_trace.h/.c** - CANopen trace object for recording variables over time.
 - **example/** - Directory with basic example, should compile on any system.
   - **CO_driver_target.h** - Example hardware definitions for CANopenNode.
   - **CO_driver_blank.c** - Example blank interface for CANopenNode.
   - **main_blank.c** - Mainline and other threads - example template.
   - **CO_storageBlank.h/.c** - Example blank demonstration for data storage to non-volatile memory.
   - **Makefile** - Makefile for example.
   - **DS301_profile.xpd** - CANopen device description file for DS301. It includes also CANopenNode specific properties. This file is also available in Profiles in Object dictionary editor.
   - **DS301_profile.eds**, **DS301_profile.md** - Standard CANopen EDS file and markdown documentation file, automatically generated from DS301_profile.xpd.
   - **OD.h/.c** - CANopen Object dictionary source files, automatically generated from DS301_profile.xpd.
 - **doc/** - Directory with documentation
   - **CHANGELOG.md** - Change Log file.
   - **deviceSupport.md** - Information about supported devices.
   - **objectDictionary.md** - Description of CANopen object dictionary interface.
   - **CANopenNode.png** - Little icon.
   - **html** - Directory with documentation - must be generated by Doxygen.
 - **CANopen.h/.c** - Initialization and processing of CANopen objects, suitable for common configurations.
 - **codingStyle** - Example of the coding style.
 - **Doxyfile** - Configuration file for the documentation generator *doxygen*.
 - **LICENSE** - License.
 - **README.md** - This file.


Object dictionary editor
------------------------
Object Dictionary is one of the most essential parts of CANopen.

To customize the Object Dictionary it is necessary to use external application: [CANopenEditor](https://github.com/CANopenNode/CANopenEditor). Latest pre-compiled [binaries](https://github.com/CANopenNode/CANopenEditor/archive/refs/heads/build.zip) are also available. Just extract the zip file and run the `EDSEditor.exe`. In Linux it runs with mono, which is available by default on Ubuntu. Just set file permissions to "executable" and then execute the program.

In program, in preferences, set exporter to "CANopenNode_V4". Then start new project or open the existing project file.

Many project file types are supported, EDS, XDD v1.0, XDD v1.1, old custom XML format. Generated project file can then be saved in XDD v1.1 file format (xmlns="http://www.canopen.org/xml/1.1"). Project file can also be exported to other formats, it can be used to generate documentation and CANopenNode source files for Object Dictionary.

If new project was started, then `DS301_profile.xpd` may be inserted. If existing (old) project is edited, then existing `Communication Specific Parameters` may be deleted and then new `DS301_profile.xpd` may be inserted. Alternative is editing existing communication parameters with observation to Object Dictionary Requirements By CANopenNode in [objectDictionary.md](doc/objectDictionary.md).

To clone, add or delete, select object(s) and use right click. Some knowledge of CANopen is required to correctly set-up the custom Object Dictionary. Separate objects can also be inserted from another project.

CANopenNode includes some custom properties inside standard project file. See [objectDictionary.md](doc/objectDictionary.md) for more information.


Device support
--------------
CANopenNode can run on many different devices. Each device (or microcontroller) must have own interface to CANopenNode. CANopenNode can run with or without operating system.

It is not practical to have all device interfaces in a single project. Interfaces to other microcontrollers are in separate projects. See [deviceSupport.md](doc/deviceSupport.md) for list of known device interfaces.


Some details
------------
### RTR
RTR (remote transmission request) is a feature of CAN bus. Usage of RTR is not recommended for CANopen and it is not implemented in CANopenNode.

### Error control
When node is started (in NMT operational state), it is allowed to send or receive Process Data Objects (PDO). If Error Register (object 0x1001) is set, then NMT operational state may not be allowed.

### Power saving
All CANopen objects calculates next timer info for OS. Calculation is based on various timers which expire in known time. Can be used to put microcontroller into sleep and wake at the calculated time.


Change Log
----------
See [CHANGELOG.md](doc/CHANGELOG.md)


License
-------
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
