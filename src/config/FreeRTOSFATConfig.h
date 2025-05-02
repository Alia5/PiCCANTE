
#ifndef FreeRTOS_FAT_CONFIG_H
#define FreeRTOS_FAT_CONFIG_H

#define portINLINE inline

#define ffconfigBYTE_ORDER pdFREERTOS_LITTLE_ENDIAN
#define ipconfigQUICK_SHORT_FILENAME_CREATION 1
#define ffconfigREMOVABLE_MEDIA 1
#define ffconfigINLINE_MEMORY_ACCESS 1
#define ffconfigCWD_THREAD_LOCAL_INDEX 1
#define ffconfigHAS_CWD 1
#define ffconfigCACHE_WRITE_THROUGH 1
#define ffconfigMAX_PARTITIONS 1
#define ffconfig64_NUM_SUPPORT 1

#endif