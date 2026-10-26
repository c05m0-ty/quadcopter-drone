    AREA |.text|, CODE, READONLY, ALIGN=2
    THUMB
    PRESERVE8

    EXTERN fault_handler_c

HardFault_Handler PROC
    EXPORT HardFault_Handler
    TST lr, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    B fault_handler_c
    ENDP

MemManage_Handler PROC
    EXPORT MemManage_Handler
    TST lr, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    B fault_handler_c
    ENDP

BusFault_Handler PROC
    EXPORT BusFault_Handler
    TST lr, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    B fault_handler_c
    ENDP

UsageFault_Handler PROC
    EXPORT UsageFault_Handler
    TST lr, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    B fault_handler_c
    ENDP

    END
