/**
 * Main CANopenNode file.
 *
 * @file        CANopen.h
 * @ingroup     CO_CANopen
 * @author      Janez Paternoster
 * @author      Uwe Kindler
 * @copyright   2010 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef CANopen_H
#define CANopen_H

#include "301/CO_driver.h"
#include "301/CO_ODinterface.h"
#include "301/CO_NMT_Heartbeat.h"
#include "301/CO_HBconsumer.h"
#include "301/CO_Emergency.h"
#include "301/CO_SDOserver.h"
#include "301/CO_SDOclient.h"
#include "301/CO_SYNC.h"
#include "301/CO_PDO.h"
#include "301/CO_TIME.h"
#include "303/CO_LEDs.h"
#include "304/CO_GFC.h"
#include "304/CO_SRDO.h"
#include "305/CO_LSSslave.h"
#include "305/CO_LSSmaster.h"
#include "309/CO_gateway_ascii.h"
#include "extra/CO_trace.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_CANopen CANopen
 * @{
 *
 * CANopenNode is free and open source CANopen communication protocol stack.
 *
 * CANopen is the internationally standardized (EN 50325-4) (CiA DS-301)
 * CAN-based higher-layer protocol for embedded control system. For more
 * information on CANopen see http://www.can-cia.org/
 *
 * CANopenNode homepage is https://github.com/CANopenNode/CANopenNode
 *
 * CANopen.h 파일은 모든 CANopenNode 소스 파일을 결합시킨다.
 * CO_STACK_CONFIG 는 "CO_config.h" 파일 내에서 먼저 정의된다. 
 * 만약 단일 object dictionary 정의를 사용한다면, 사용하는 다양한 CANopenNode objects는 CO_config_t 구조체로 설정되거나 "OD.h" 파일에서 직접 읽는다. 
 * "OD.h" 와 "OD.c" 파일은 CANopen Object Dictionary를 정의하고 외부 도구에 의해 생성된다.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * @}
 */

/**
 * @defgroup CO_CANopen_301 CANopen_301
 * @{
 *
 * CANopen application layer and communication profile (CiA 301 v4.2.0)
 *
 * data types, encoding 법칙, object dictionary objects, CANopen communication services와 protocols를 정의한다.
 * 
 * Definitions of data types, encoding rules, object dictionary objects and
 * CANopen communication services and protocols.
 * @}
 */

/**
 * @defgroup CO_CANopen_303 CANopen_303
 * @{
 *
 * CANopen recommendation for indicator specification (CiA 303-3 v1.4.0)
 *
 * indicators와 관련된 통신 설명 - 녹색과 빨간색 LED 다이오드
 * Description of communication related indicators - green and red LED diodes.
 * @}
 */

/**
 * @defgroup CO_CANopen_304 CANopen_304
 * @{
 *
 * CANopen Safety (EN 50325­-5:2010 (formerly CiA 304))
 *
 * Standard defines the usage of Safety Related Data Objects (SRDO) and the GFC.
 * This is an additional protocol (to SDO, PDO) to exchange data. The meaning of
 * "security" here refers not to security (crypto) but to data consistency.
 * @}
 */

/**
 * @defgroup CO_CANopen_305 CANopen_305
 * @{
 *
 * CANopen layer setting services (LSS) and protocols (CiA 305 DSP v3.0.0)
 *
 * Inquire or change three parameters on a CANopen device with LSS slave
 * capability by a CANopen device with LSS master capability via the CAN
 * network: the settings of Node-ID of the CANopen device, bit timing
 * parameters of the physical layer (bit rate) or LSS address compliant to the
 * identity object (1018h).
 * @}
 */

/**
 * @defgroup CO_CANopen_309 CANopen_309
 * @{
 *
 * CANopen access from other networks (CiA 309)
 *
 * Standard defines the services and protocols to interface CANopen networks to
 * other networks. Standard is organized as follows:
 * - Part 1: General principles and services
 * - Part 2: Modbus/TCP mapping
 * - Part 3: ASCII mapping
 * - Part 4: Amendment 7 to Fieldbus Integration into PROFINET IO
 * @}
 */

/**
 * @defgroup CO_CANopen_storage CANopen_storage
 * @{
 *
 * CANopen Object Dictionary and other data storage.
 * @}
 */

/**
 * @defgroup CO_CANopen_extra CANopen_extra
 * @{
 *
 * Additional non-standard objects related to CANopenNode.
 * @}
 */

/**
 * @addtogroup CO_CANopen
 * @{
 */

/**
 * If macro is defined externally, then configuration with multiple object
 * dictionaries will be possible. If macro is not defined, default "OD.h" file
 * with necessary definitions, such as OD_CNT_xxx, will be used, and also memory
 * consumption and startup time will be lower.
 */
#ifdef CO_DOXYGEN
#define CO_MULTIPLE_OD
#endif

/**
 * If macro is defined externally, then global variables for CANopen objects
 * will be used instead of heap. This is possible only if CO_MULTIPLE_OD is not
 * defined.
 */
#ifdef CO_DOXYGEN
#define CO_USE_GLOBALS
#endif


#if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
/**
 * CANopen configuration, used with @ref CO_new()
 *
 * This structure is used only, if @ref CO_MULTIPLE_OD is enabled. Otherwise
 * parameters are retrieved from default "OD.h" file.
 */
typedef struct {
    /** Number of NMT objects, 0 or 1: NMT slave (CANrx) + Heartbeat producer
     * (CANtx) + optional NMT master (CANtx), configurable by
     * @ref CO_CONFIG_NMT. Start indexes inside CANrx and CANtx are always 0.
     * There must be one NMT object in the device. */
    uint8_t CNT_NMT;
    OD_entry_t *ENTRY_H1017; /**< OD entry for @ref CO_NMT_init() */
    /** Number of Heartbeat consumer objects, 0 or 1 */
    uint8_t CNT_HB_CONS;
    /** Number of internal consumers (CANrx), used inside Heartbeat consumer
     * object, 1 to 127. */
    uint8_t CNT_ARR_1016;
    OD_entry_t *ENTRY_H1016; /**< OD entry for @ref CO_HBconsumer_init()*/
    /** Number of Emergency objects, 0 or 1: optional producer (CANtx) +
     * optional consumer (CANrx), configurable by @ref CO_CONFIG_EM.
     * There must be one Emergency object in the device. */
    uint8_t CNT_EM;
    const OD_entry_t *ENTRY_H1001; /**< OD entry for @ref CO_EM_init() */
    OD_entry_t *ENTRY_H1014; /**< OD entry for @ref CO_EM_init() */
    OD_entry_t *ENTRY_H1015; /**< OD entry for @ref CO_EM_init() */
    /** Size of the fifo buffer, which is used for intermediate storage of
     * emergency messages. Fifo is used by emergency producer and by error
     * history (OD object 0x1003). Size is usually equal to size of array in
     * OD object 0x1003. If later is not used, CNT_ARR_1003 must also be set to
     * value greater than 0, or emergency producer will not work. */
    uint8_t CNT_ARR_1003;
    OD_entry_t *ENTRY_H1003; /**< OD entry for @ref CO_EM_init() */
    /** Number of SDO server objects, from 0 to 128 (CANrx + CANtx). There must
     * be at least one SDO server object in the device. */
    uint8_t CNT_SDO_SRV;
    OD_entry_t *ENTRY_H1200; /**< OD entry for @ref CO_SDOserver_init()*/
    /** Number of SDO client objects, from 0 to 128 (CANrx + CANtx). */
    uint8_t CNT_SDO_CLI;
    OD_entry_t *ENTRY_H1280; /**< OD entry for @ref CO_SDOclient_init()*/
    /** Number of TIME objects, 0 or 1: consumer (CANrx) + optional producer
     * (CANtx), configurable by @ref CO_CONFIG_TIME. */
    uint8_t CNT_TIME;
    OD_entry_t *ENTRY_H1012; /**< OD entry for @ref CO_TIME_init() */
    /** Number of SYNC objects, 0 or 1:  consumer (CANrx) + optional producer
     * (CANtx), configurable by @ref CO_CONFIG_SYNC. */
    uint8_t CNT_SYNC;
    OD_entry_t *ENTRY_H1005; /**< OD entry for @ref CO_SYNC_init() */
    OD_entry_t *ENTRY_H1006; /**< OD entry for @ref CO_SYNC_init() */
    OD_entry_t *ENTRY_H1007; /**< OD entry for @ref CO_SYNC_init() */
    OD_entry_t *ENTRY_H1019; /**< OD entry for @ref CO_SYNC_init() */
    /** Number of RPDO objects, from 0 to 512 consumers (CANrx) */
    uint16_t CNT_RPDO;
    OD_entry_t *ENTRY_H1400; /**< OD entry for @ref CO_RPDO_init() */
    OD_entry_t *ENTRY_H1600; /**< OD entry for @ref CO_RPDO_init() */
    /** Number of TPDO objects, from 0 to 512 producers (CANtx) */
    uint16_t CNT_TPDO;
    OD_entry_t *ENTRY_H1800; /**< OD entry for @ref CO_TPDO_init() */
    OD_entry_t *ENTRY_H1A00; /**< OD entry for @ref CO_TPDO_init() */
    /** Number of LEDs objects, 0 or 1. */
    uint8_t CNT_LEDS;
    /** Number of GFC objects, 0 or 1 (CANrx + CANtx). */
    uint8_t CNT_GFC;
    OD_entry_t *ENTRY_H1300; /**< OD entry for @ref CO_GFC_init() */
    /** Number of SRDO objects, from 0 to 64 (2*CANrx + 2*CANtx). */
    uint8_t CNT_SRDO;
    OD_entry_t *ENTRY_H1301; /**< OD entry for @ref CO_SRDO_init() */
    OD_entry_t *ENTRY_H1381; /**< OD entry for @ref CO_SRDO_init() */
    OD_entry_t *ENTRY_H13FE; /**< OD entry for @ref CO_SRDOGuard_init() */
    OD_entry_t *ENTRY_H13FF; /**< OD entry for @ref CO_SRDOGuard_init() */
    /** Number of LSSslave objects, 0 or 1 (CANrx + CANtx). */
    uint8_t CNT_LSS_SLV;
    /** Number of LSSmaster objects, 0 or 1 (CANrx + CANtx). */
    uint8_t CNT_LSS_MST;
    /** Number of gateway ascii objects, 0 or 1. */
    uint8_t CNT_GTWA;
    /** Number of trace objects, 0 or more. */
    uint16_t CNT_TRACE;
} CO_config_t;
#else
typedef void CO_config_t;
#endif /* CO_MULTIPLE_OD */


/**
 * CANopen object - 모든 CANopenNode object의 모임
 */
typedef struct {
    bool_t nodeIdUnconfigured; /**< True in un-configured LSS slave */
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    CO_config_t *config; /**< 설정 파라미터 기억 */
 #endif
    /** One CAN module object, @ref CO_CANmodule_init() 에서 초기화 */
    CO_CANmodule_t *CANmodule;
    CO_CANrx_t *CANrx; /**< CAN 수신 메시지 objects */
    CO_CANtx_t *CANtx; /**< CAN 전송 메시지 objects */
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t CNT_ALL_RX_MSGS; /**< 모든 CAN 수신 메시지 objects의 개수  */
    uint16_t CNT_ALL_TX_MSGS; /**< 모든 CAN 전송 메시지 objects의 개수  */
 #endif
    /** NMT 와 heartbeat object, @ref CO_NMT_init() 에서 초기화 */
    CO_NMT_t *NMT;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_NMT_SLV; /**< 시작 index in CANrx. */
    uint16_t TX_IDX_NMT_MST; /**< 시작 index in CANtx. */
    uint16_t TX_IDX_HB_PROD; /**< 시작 index in CANtx. */
 #endif
#if ((CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE) || defined CO_DOXYGEN
    /** Heartbeat consumer object, @ref CO_HBconsumer_init() 에서 초기화 */
    CO_HBconsumer_t *HBcons;
    /** Object for monitored nodes, @ref CO_HBconsumer_init() 에서 초기화 */
    CO_HBconsNode_t *HBconsMonitoredNodes;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_HB_CONS; /**< 시작 index in CANrx. */
 #endif
#endif
    /** Emergency object, @ref CO_EM_init() 에서 초기화 */
    CO_EM_t *em;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_EM_CONS; /**< Start index in CANrx. */
    uint16_t TX_IDX_EM_PROD; /**< Start index in CANtx. */
 #endif
#if ((CO_CONFIG_EM) & (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_HISTORY)) \
    || defined CO_DOXYGEN
    /** FIFO for emergency object, @ref CO_EM_init() 에서 초기화 */
    CO_EM_fifo_t *em_fifo;
#endif
    /** SDO server objects, @ref CO_SDOserver_init() 에서 초기화 */
    CO_SDOserver_t *SDOserver;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_SDO_SRV; /**< 시작 index in CANrx. */
    uint16_t TX_IDX_SDO_SRV; /**< 시작 index in CANtx. */
 #endif
#if ((CO_CONFIG_SDO_CLI) & CO_CONFIG_SDO_CLI_ENABLE) || defined CO_DOXYGEN
    /** SDO client objects, @ref CO_SDOclient_init() 에서 초기화 */
    CO_SDOclient_t *SDOclient;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_SDO_CLI; /**< Start index in CANrx. */
    uint16_t TX_IDX_SDO_CLI; /**< Start index in CANtx. */
 #endif
#endif
#if ((CO_CONFIG_TIME) & CO_CONFIG_TIME_ENABLE) || defined CO_DOXYGEN
    /** TIME object, @ref CO_TIME_init() 에서 초기화 */
    CO_TIME_t *TIME;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_TIME; /**< Start index in CANrx. */
    uint16_t TX_IDX_TIME; /**< Start index in CANtx. */
 #endif
#endif
#if ((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE) || defined CO_DOXYGEN
    /** SYNC object, @ref CO_SYNC_init() 에서 초기화 */
    CO_SYNC_t *SYNC;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_SYNC; /**< Start index in CANrx. */
    uint16_t TX_IDX_SYNC; /**< Start index in CANtx. */
 #endif
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE) || defined CO_DOXYGEN
    /** RPDO objects, @ref CO_RPDO_init() 에서 초기화 */
    CO_RPDO_t *RPDO;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_RPDO; /**< Start index in CANrx. */
 #endif
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE) || defined CO_DOXYGEN
    /** TPDO objects, @ref CO_TPDO_init() 에서 초기화 */
    CO_TPDO_t *TPDO;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t TX_IDX_TPDO; /**< Start index in CANtx. */
 #endif
#endif
#if ((CO_CONFIG_LEDS) & CO_CONFIG_LEDS_ENABLE) || defined CO_DOXYGEN
    /** LEDs object, @ref CO_LEDs_init() 에서 초기화 */
    CO_LEDs_t *LEDs;
#endif
#if ((CO_CONFIG_GFC) & CO_CONFIG_GFC_ENABLE) || defined CO_DOXYGEN
    /** GFC object, @ref CO_GFC_init() 에서 초기화 */
    CO_GFC_t *GFC;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_GFC; /**< Start index in CANrx. */
    uint16_t TX_IDX_GFC; /**< Start index in CANtx. */
 #endif
#endif
#if ((CO_CONFIG_SRDO) & CO_CONFIG_SRDO_ENABLE) || defined CO_DOXYGEN
    /** SRDO object, initialised by @ref CO_SRDOGuard_init(), single SRDOGuard
     * object is included inside all SRDO objects */
    CO_SRDOGuard_t *SRDOGuard;
    /** SRDO objects, @ref CO_SRDO_init() 에서 초기화 */
    CO_SRDO_t *SRDO;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_SRDO; /**< Start index in CANrx. */
    uint16_t TX_IDX_SRDO; /**< Start index in CANtx. */
 #endif
#endif
#if ((CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE) || defined CO_DOXYGEN
    /** LSS slave object, @ref CO_LSSslave_init() 에서 초기화 */
    CO_LSSslave_t *LSSslave;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_LSS_SLV; /**< Start index in CANrx. */
    uint16_t TX_IDX_LSS_SLV; /**< Start index in CANtx. */
 #endif
#endif
#if ((CO_CONFIG_LSS) & CO_CONFIG_LSS_MASTER) || defined CO_DOXYGEN
    /** LSS master object, @ref CO_LSSmaster_init() 에서 초기화 */
    CO_LSSmaster_t *LSSmaster;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
    uint16_t RX_IDX_LSS_MST; /**< Start index in CANrx. */
    uint16_t TX_IDX_LSS_MST; /**< Start index in CANtx. */
 #endif
#endif
#if ((CO_CONFIG_GTW) & CO_CONFIG_GTW_ASCII) || defined CO_DOXYGEN
    /** Gateway-ascii object, @ref CO_GTWA_init() 에서 초기화 */
    CO_GTWA_t *gtwa;
 #if defined CO_MULTIPLE_OD || defined CO_DOXYGEN
 #endif
#endif
#if ((CO_CONFIG_TRACE) & CO_CONFIG_TRACE_ENABLE) || defined CO_DOXYGEN
    /** Trace object, @ref CO_trace_init() 에서 초기화 */
    CO_trace_t *trace;
#endif
} CO_t;


/**
 * 새로운 CANopen object 생성
 *
 * CO_USE_GLOBALS가 정의되어 있다면, 모든 CANopenNode objects에 대해서 함수는 global static 변수를 사용한다.
 * 만약 정의되어 있지 않으면 heap에서 모든 objects를 할당한다.
 *
 * @remark
 * With some microcontrollers it is necessary to specify Heap size within
 * linker configuration, if heap is used.
 *
 * @param config Configuration structure, used if @ref CO_MULTIPLE_OD is
 * defined. It must stay in memory permanently. If CO_MULTIPLE_OD is not
 * defined, config should be NULL and parameters are retrieved from default
 * "OD.h" file.
 * @param [out] heapMemoryUsed Information about heap memory used. Ignored if
 * NULL.
 *
 * @return Successfully allocated and configured CO_t object or NULL.
 */
CO_t *CO_new(CO_config_t *config, uint32_t *heapMemoryUsed);


/**
 * CANopen object 삭제 및 메모리 해제. 프로그램 종료시 호출되어야 한다.
 *
 * @param co CANopen object.
 */
void CO_delete(CO_t *co);


/**
 * LSS slave가 활성화되어 있는지 테스트
 *
 * @param co CANopen object.
 *
 * @return True if enabled
 */
bool_t CO_isLSSslaveEnabled(CO_t *co);


/**
 * CAN driver를 초기화
 *
 * communication reset setion 낸에서 호출되어야만 한다.
 *
 * @param co CANopen object.
 * @param CANptr Pointer to the user-defined CAN base structure, passed to
 *               CO_CANmodule_init().
 * @param bitRate CAN bit rate.
 * @return CO_ERROR_NO in case of success.
 */
CO_ReturnError_t CO_CANinit(CO_t *co, void *CANptr, uint16_t bitRate);


#if ((CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE) || defined CO_DOXYGEN
/**
 * CANopen LSS slave를 초기화
 *
 * CO_CANopenInit 전에 호출되어야만 한다.
 *
 * See @ref CO_LSSslave_init() for description of parameters.
 *
 * @param co CANopen object.
 * @param lssAddress LSS slave address, from OD object 0x1018
 * @param [in,out] pendingNodeID Pending node ID or 0xFF (unconfigured)
 * @param [in,out] pendingBitRate Pending bit rate of the CAN interface
 *
 * @return CO_ERROR_NO in case of success.
 */
CO_ReturnError_t CO_LSSinit(CO_t *co,
                            CO_LSS_address_t *lssAddress,
                            uint8_t *pendingNodeID,
                            uint16_t *pendingBitRate);
#endif


/**
 * PDO objects를 제외한 CANopenNode 초기화
 *
 * communication reset section 내에서 호출되어야만 한다.
 *
 * @param co CANopen object.
 * @param em Emergency object, which is used inside different CANopen objects,
 * usually for error reporting. If NULL, then 'co->em' will be used.
 * if NULL and 'co->CNT_EM' is 0, then function returns with error.
 * @param NMT If 'co->CNT_NMT' is 0, this object must be specified, If
 * 'co->CNT_NMT' is 1,then it is ignored and can be NULL. NMT object is used for
 * retrieving NMT internal state inside CO_process().
 * @param od CANopen Object dictionary
 * @param OD_statusBits Argument passed to @ref CO_EM_init(). May be NULL.
 * @param NMTcontrol Argument passed to @ref CO_NMT_init().
 * @param firstHBTime_ms Argument passed to @ref CO_NMT_init().
 * @param SDOserverTimeoutTime_ms Argument passed to @ref CO_SDOserver_init().
 * @param SDOclientTimeoutTime_ms Default timeout in milliseconds for SDO
 * client, 500 typically. SDO client is configured from CO_GTWA_init().
 * @param SDOclientBlockTransfer If true, block transfer will be set in SDO
 * client by default. SDO client is configured from by CO_GTWA_init().
 * @param nodeId CANopen Node ID (1 ... 127) or 0xFF(unconfigured). In the
 * CANopen initialization it is the same as pendingBitRate from CO_LSSinit().
 * If it is unconfigured, then some CANopen objects will not be initialized nor
 * processed.
 * @param [out] errInfo Additional information in case of error, may be NULL.
 * errInfo can also be set in noncritical errors, where function returns
 * CO_ERROR_NO. For example, if OD parameter contains wrong value.
 *
 * @return CO_ERROR_NO in case of success.
 */
CO_ReturnError_t CO_CANopenInit(CO_t *co,
                                CO_NMT_t *NMT,
                                CO_EM_t *em,
                                OD_t *od,
                                OD_entry_t *OD_statusBits,
                                CO_NMT_control_t NMTcontrol,
                                uint16_t firstHBTime_ms,
                                uint16_t SDOserverTimeoutTime_ms,
                                uint16_t SDOclientTimeoutTime_ms,
                                bool_t SDOclientBlockTransfer,
                                uint8_t nodeId,
                                uint32_t *errInfo);


/**
 * CANopenNode PDO objects 를 초기화.
 *
 * 모든 CANopenNode objects가 초기화된 후에 communication reset section의 마지막에 호출되어야만 한다.
 * 그렇지 않으면 일부 OD 변수가 PDO와 제대로 매핑되지 않을 수 있다.
 *
 * @param co CANopen object.
 * @param em Emergency object, which is used inside PDO objects for error
 * reporting.
 * @param od CANopen Object dictionary
 * @param nodeId CANopen Node ID (1 ... 127) or 0xFF(unconfigured). If
 * unconfigured, then PDO will not be initialized nor processed.
 * @param [out] errInfo Additional information in case of error, may be NULL.
 *
 * @return CO_ERROR_NO in case of success.
 */
CO_ReturnError_t CO_CANopenInitPDO(CO_t *co,
                                   CO_EM_t *em,
                                   OD_t *od,
                                   uint8_t nodeId,
                                   uint32_t *errInfo);


/**
 * Process CANopen objects.
 *
 * 주기적으로 호출되어야만 한다. 모든 "비동기" CANopen objects를 처리한다.
 *
 * @param co CANopen object.
 * @param enableGateway If true, gateway to external world will be enabled.
 * @param timeDifference_us Time difference from previous function call in
 *                          microseconds.
 * @param [out] timerNext_us info to OS - maximum delay time after this function
 *        should be called next time in [microseconds]. Value can be used for OS
 *        sleep time. Initial value must be set to maximum interval time.
 *        Output will be equal or lower to initial value. Calculation is based
 *        on various timers which expire in known time. Parameter should be
 *        used in combination with callbacks configured with
 *        CO_***_initCallbackPre() functions. Those callbacks should also
 *        trigger calling of CO_process() function. Parameter is ignored if
 *        NULL. See also @ref CO_CONFIG_FLAG_CALLBACK_PRE configuration macro.
 *
 * @return Node or communication reset request, from @ref CO_NMT_process().
 */
CO_NMT_reset_cmd_t CO_process(CO_t *co,
                              bool_t enableGateway,
                              uint32_t timeDifference_us,
                              uint32_t *timerNext_us);


#if ((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE) || defined CO_DOXYGEN
/**
 * Process CANopen SYNC objects.
 *
 * 주기적으로 호출되어야만 한다. time critical application에 대해서 일정한 주기(일반적으로 1ms)를 가지는 실시간 thread에서 호출될 수 있다.
 * 여기서 SYNC CANopen objects를 처리한다.
 *
 * @param co CANopen object.
 * @param timeDifference_us Time difference from previous function call in
 * microseconds.
 * @param [out] timerNext_us info to OS - see CO_process().
 *
 * @return True, if CANopen SYNC message was just received or transmitted.
 */
bool_t CO_process_SYNC(CO_t *co,
                       uint32_t timeDifference_us,
                       uint32_t *timerNext_us);
#endif


#if ((CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE) || defined CO_DOXYGEN
/**
 * Process CANopen RPDO objects.
 *
 * 주기적으로 호출되어야만 한다. time critical application에 대해서 일정한 주기(일반적으로 1ms)를 가지는 실시간 thread에서 호출될 수 있다.
 * receive PDO CANopen objects를 처리한다.
 *
 * @param co CANopen object.
 * @param syncWas True, if CANopen SYNC message was just received or
 * transmitted.
 * @param timeDifference_us Time difference from previous function call in
 * microseconds.
 * @param [out] timerNext_us info to OS - see CO_process().
 */
void CO_process_RPDO(CO_t *co,
                     bool_t syncWas,
                     uint32_t timeDifference_us,
                     uint32_t *timerNext_us);
#endif


#if ((CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE) || defined CO_DOXYGEN
/**
 * Process CANopen TPDO objects.
 *
 * 주기적으로 호출되어야만 한다. time critical application에 대해서 일정한 주기(일반적으로 1ms)를 가지는 실시간 thread에서 호출될 수 있다.
 * 여기서 transmit PDO CANopen objects를 처리한다.
 *
 * @param co CANopen object.
 * @param syncWas True, if CANopen SYNC message was just received or
 * transmitted.
 * @param timeDifference_us Time difference from previous function call in
 * microseconds.
 * @param [out] timerNext_us info to OS - see CO_process().
 */
void CO_process_TPDO(CO_t *co,
                     bool_t syncWas,
                     uint32_t timeDifference_us,
                     uint32_t *timerNext_us);
#endif


#if ((CO_CONFIG_SRDO) & CO_CONFIG_SRDO_ENABLE) || defined CO_DOXYGEN
/**
 * Process CANopen SRDO objects.
 *
 * 주기적으로 호출되어야만 한다. time critical application에 대해서 일정한 주기(일반적으로 1ms)를 가지는 실시간 thread에서 호출될 수 있다.
 * 여기서 SRDO CANopen objects를 처리한다.
 *
 * @param co CANopen object.
 * @param timeDifference_us Time difference from previous function call in
 * microseconds.
 * @param [out] timerNext_us info to OS - see CO_process().
 */
void CO_process_SRDO(CO_t *co,
                     uint32_t timeDifference_us,
                     uint32_t *timerNext_us);
#endif

/** @} */ /* CO_CANopen */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CANopen_H */
