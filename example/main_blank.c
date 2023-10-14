/*
 * CANopen main program file.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        main_generic.c
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
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


#include <stdio.h>

#include "CANopen.h"
#include "OD.h"
#include "CO_storageBlank.h"


#define log_printf(macropar_message, ...) \
        printf(macropar_message, ##__VA_ARGS__)


/* CO_CANopenInit()를 위한 기본값 */
#define NMT_CONTROL \
            CO_NMT_STARTUP_TO_OPERATIONAL \
          | CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL


/* Global 변수와 객체 */
CO_t *CO = NULL; /* CANopen object */
uint8_t LED_red, LED_green;


/* main ***********************************************************************/
int main (void){
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    void *CANptr = NULL; /* CAN module 주소 */
    uint8_t pendingNodeId = 10; /* dip 스위치, non-volatile 메모리, LSS slave로부터 읽기 */
    uint8_t activeNodeId = 10; /* communication reset section내에서 CO_pendingNodeId에서 복사하기 */
    uint16_t pendingBitRate = 125;  /* dip 스위치, non-volatile 메모리, LSS slave로부터 읽기 */

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storage_t storage;
    CO_storage_entry_t storageEntries[] = {
        {
            .addr = &OD_PERSIST_COMM,
            .len = sizeof(OD_PERSIST_COMM),
            .subIndexOD = 2,
            .attr = CO_storage_cmd | CO_storage_restore,
            .addrNV = NULL
        }
    };
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0;
#endif

    /* microcontroller 설정 */


    /* 메모리 할당 */
    CO_config_t *config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    /* example usage of CO_MULTIPLE_OD (but still single OD here) */
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
    co_config.CNT_LEDS = 1;
    co_config.CNT_LSS_SLV = 1;
    config_ptr = &co_config;
#endif /* CO_MULTIPLE_OD */
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return 0;
    }
    else {
        log_printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
    }


#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage,
                               CO->CANmodule,
                               OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters,
                               storageEntries,
                               storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", storageInitError);
        return 0;
    }
#endif


    while(reset != CO_RESET_APP){
/* CANopen communication reset - CANopen objects 초기화 *******************/
        log_printf("CANopenNode - Reset communication...\n");

        /* rt_thread 기다리기. */
        CO->CANmodule->CANnormal = false;

        /* CAN 설정 진입 */
        CO_CANsetConfigurationMode((void *)&CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        /* CANopen 초기화 */
        err = CO_CANinit(CO, CANptr, pendingBitRate);
        if (err != CO_ERROR_NO) {
            log_printf("Error: CAN initialization failed: %d\n", err);
            return 0;
        }

        CO_LSS_address_t lssAddress = {.identity = {
            .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
            .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
            .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
            .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
        }};
        err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate);
        if(err != CO_ERROR_NO) {
            log_printf("Error: LSS slave initialization failed: %d\n", err);
            return 0;
        }

        activeNodeId = pendingNodeId;
        uint32_t errInfo = 0;

        err = CO_CANopenInit(CO,                /* CANopen object */
                             NULL,              /* alternate NMT */
                             NULL,              /* alternate em */
                             OD,                /* Object dictionary */
                             OD_STATUS_BITS,    /* Optional OD_statusBits */
                             NMT_CONTROL,       /* CO_NMT_control_t */
                             FIRST_HB_TIME,     /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
                             activeNodeId,
                             &errInfo);
        if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
            }
            else {
                log_printf("Error: CANopen initialization failed: %d\n", err);
            }
            return 0;
        }

        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if(err != CO_ERROR_NO) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
            }
            else {
                log_printf("Error: PDO initialization failed: %d\n", err);
            }
            return 0;
        }

        /* 1ms 마다 실행시키기 위해서 타이머 interrupt 함수 설정 */


        /* CAN 전송 및 수신 interrupt 설정 */


        /* CANopen callbacks 설정 등등 */
        if(!CO->nodeIdUnconfigured) {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            if(storageInitError != 0) {
                CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY,
                               CO_EMC_HARDWARE, storageInitError);
            }
#endif
        }
        else {
            log_printf("CANopenNode - Node-id not initialized\n");
        }


        /* CAN 구동시키기 */
        CO_CANsetNormalMode(CO->CANmodule);

        reset = CO_RESET_NOT;

        log_printf("CANopenNode - Running...\n");
        fflush(stdout);

        while(reset == CO_RESET_NOT){
/* 일반 program 실행을 위한 loop ******************************************/
            /* 마지막 함수 호출 이후 시간 차이 얻기  */
            uint32_t timeDifference_us = 500;

            /* CANopen process */
            reset = CO_process(CO, false, timeDifference_us, NULL);
            LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
            LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

            /* Nonblocking application 코드가 여기 올 수 있음 */

            /* Process automatic storage */

            /* 옵션 : sleep for short time */
        }
    }


/* program 종료 ***************************************************************/
    /* stop threads */


    /* delete objects from memory */
    CO_CANsetConfigurationMode((void *)&CANptr);
    CO_delete(CO);

    log_printf("CANopenNode finished\n");

    /* reset */
    return 0;
}


/* 일정한 주기로 timer thread 실행  ********************************/
void tmrTask_thread(void){

    for(;;) {
        CO_LOCK_OD(CO->CANmodule);
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
            bool_t syncWas = false;
            /* get time difference since last function call */
            uint32_t timeDifference_us = 1000;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

            /* 추가적인 I/O 혹은 nonblocking application code가 여기 올 수 있음  */
        }
        CO_UNLOCK_OD(CO->CANmodule);
    }
}


/* CAN interrupt function는 수신한 CAN message에 대해서 실행 ********************/
void /* interrupt */ CO_CAN1InterruptHandler(void){

    /* clear interrupt flag */
}
