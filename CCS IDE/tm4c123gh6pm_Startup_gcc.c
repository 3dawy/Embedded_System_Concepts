/**
Start-up code for ARM Cortex-M4 MCU on Tiva-C LaunchPad board using
tm4c123gh6pm MCU.

Startup code for most other processors written in assembly, because SP not
set up yet and .data not copy from ROM to RAM, and .bss isn't cleared yet. but
ARM Cortex-M designed to reduce the need for low-level assembly.
*/
#include <stdint.h>


//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

/* start and end of stack defined in the linker script ---------------------*/
extern int __stack_start__;
extern int __stack_end__;


/**
* \note
* The function assert_failed defined at the end of this file defines
* the error/assertion handling policy for the application and might
* need to be customized for each project. This function is defined in
* assembly to avoid accessing the stack, which might be corrupted by
* the time assert_failed is called.
*/

/*
 * "naked" functions do not include any prologue or epilogue -- they are naked.
 * In particular, they do not include operations on the stack for local variables,
 * to save or restore registers, or to return to a calling function.
 */
__attribute__ ((naked)) void assert_failed(char const *module, int loc);


//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
/* Function prototypes -----------------------------------------------------*/
void Default_Handler(void);  /* Default empty handler */
void Reset_Handler(void);    /* Reset Handler */




/*----------------------------------------------------------------------------
* weak aliases for each Exception handler to the Default_Handler.
* Any function with the same name will override these definitions.
*/
/* Cortex-M Processor fault exceptions... */
void NMI_Handler           (void) __attribute__ ((weak));
void HardFault_Handler     (void) __attribute__ ((weak));
void MemManage_Handler     (void) __attribute__ ((weak));
void BusFault_Handler      (void) __attribute__ ((weak));
void UsageFault_Handler    (void) __attribute__ ((weak));

/* Cortex-M Processor non-fault exceptions... */
void SVC_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));

/* external interrupts...   */
void GPIOPortA_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortB_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortC_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortD_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortE_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI0_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMFault_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMGen0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMGen1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMGen2_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void QEI0_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void ADCSeq0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void ADCSeq1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void ADCSeq2_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void ADCSeq3_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Watchdog_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer0A_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer0B_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer1A_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer1B_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer2A_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer2B_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Comp0_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void Comp1_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void Comp2_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SysCtrl_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void FlashCtrl_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortF_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortG_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortH_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer3A_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer3B_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void QEI1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN0_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void Hibernate_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void USB0_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMGen3_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void uDMAST_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void uDMAError_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq2_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq3_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortJ_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortK_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortL_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI3_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART3_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void UART6_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void UART7_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer4A_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer4B_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer5A_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer5B_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer0A_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer0B_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer1A_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer1B_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer2A_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer2B_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer3A_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer3B_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer4A_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer4B_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer5A_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer5B_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C4_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C5_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortM_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortN_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void QEI2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP0_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP1_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP2_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP3_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP4_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP5_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP6_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP7_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ0_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ1_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ2_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ3_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ4_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ5_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ6_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ7_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortR_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortS_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Gen0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Gen1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Gen2_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Gen3_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Fault_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));




//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
__attribute__ ((section(".intvecs")))
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))(&__stack_end__),  // The initial stack pointer
    Reset_Handler,          /* Reset Handler                   */
    NMI_Handler,            /* NMI Handler                     */
    HardFault_Handler,      /* Hard Fault Handler              */
    MemManage_Handler,      /* The MPU fault handler           */
    BusFault_Handler,       /* The bus fault handler           */
    UsageFault_Handler,     /* The usage fault handler         */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    SVC_Handler,            /* SVCall handler                  */
    DebugMon_Handler,       /* Debug monitor handler           */
    0,                            /* Reserved                        */
    PendSV_Handler,         /* The PendSV handler              */
    SysTick_Handler,        /* The SysTick handler             */

    /*IRQ handlers... */
    GPIOPortA_IRQHandler,   /* GPIO Port A                     */
    GPIOPortB_IRQHandler,   /* GPIO Port B                     */
    GPIOPortC_IRQHandler,   /* GPIO Port C                     */
    GPIOPortD_IRQHandler,   /* GPIO Port D                     */
    GPIOPortE_IRQHandler,   /* GPIO Port E                     */
    UART0_IRQHandler,       /* UART0 Rx and Tx                 */
    UART1_IRQHandler,       /* UART1 Rx and Tx                 */
    SSI0_IRQHandler,        /* SSI0 Rx and Tx                  */
    I2C0_IRQHandler,        /* I2C0 Master and Slave           */
    PWMFault_IRQHandler,    /* PWM Fault                       */
    PWMGen0_IRQHandler,     /* PWM Generator 0                 */
    PWMGen1_IRQHandler,     /* PWM Generator 1                 */
    PWMGen2_IRQHandler,     /* PWM Generator 2                 */
    QEI0_IRQHandler,        /* Quadrature Encoder 0            */
    ADCSeq0_IRQHandler,     /* ADC Sequence 0                  */
    ADCSeq1_IRQHandler,     /* ADC Sequence 1                  */
    ADCSeq2_IRQHandler,     /* ADC Sequence 2                  */
    ADCSeq3_IRQHandler,     /* ADC Sequence 3                  */
    Watchdog_IRQHandler,    /* Watchdog timer                  */
    Timer0A_IRQHandler,     /* Timer 0 subtimer A              */
    Timer0B_IRQHandler,     /* Timer 0 subtimer B              */
    Timer1A_IRQHandler,     /* Timer 1 subtimer A              */
    Timer1B_IRQHandler,     /* Timer 1 subtimer B              */
    Timer2A_IRQHandler,     /* Timer 2 subtimer A              */
    Timer2B_IRQHandler,     /* Timer 2 subtimer B              */
    Comp0_IRQHandler,       /* Analog Comparator 0             */
    Comp1_IRQHandler,       /* Analog Comparator 1             */
    Comp2_IRQHandler,       /* Analog Comparator 2             */
    SysCtrl_IRQHandler,     /* System Control (PLL, OSC, BO)   */
    FlashCtrl_IRQHandler,   /* FLASH Control                   */
    GPIOPortF_IRQHandler,   /* GPIO Port F                     */
    GPIOPortG_IRQHandler,   /* GPIO Port G                     */
    GPIOPortH_IRQHandler,   /* GPIO Port H                     */
    UART2_IRQHandler,       /* UART2 Rx and Tx                 */
    SSI1_IRQHandler,        /* SSI1 Rx and Tx                  */
    Timer3A_IRQHandler,     /* Timer 3 subtimer A              */
    Timer3B_IRQHandler,     /* Timer 3 subtimer B              */
    I2C1_IRQHandler,        /* I2C1 Master and Slave           */
    QEI1_IRQHandler,        /* Quadrature Encoder 1            */
    CAN0_IRQHandler,        /* CAN0                            */
    CAN1_IRQHandler,        /* CAN1                            */
    CAN2_IRQHandler,        /* CAN2                            */
    0,                            /* Reserved                        */
    Hibernate_IRQHandler,   /* Hibernate                       */
    USB0_IRQHandler,        /* USB0                            */
    PWMGen3_IRQHandler,     /* PWM Generator 3                 */
    uDMAST_IRQHandler,      /* uDMA Software Transfer          */
    uDMAError_IRQHandler,   /* uDMA Error                      */
    ADC1Seq0_IRQHandler,    /* ADC1 Sequence 0                 */
    ADC1Seq1_IRQHandler,    /* ADC1 Sequence 1                 */
    ADC1Seq2_IRQHandler,    /* ADC1 Sequence 2                 */
    ADC1Seq3_IRQHandler,    /* ADC1 Sequence 3                 */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    GPIOPortJ_IRQHandler,   /* GPIO Port J                     */
    GPIOPortK_IRQHandler,   /* GPIO Port K                     */
    GPIOPortL_IRQHandler,   /* GPIO Port L                     */
    SSI2_IRQHandler,        /* SSI2 Rx and Tx                  */
    SSI3_IRQHandler,        /* SSI3 Rx and Tx                  */
    UART3_IRQHandler,       /* UART3 Rx and Tx                 */
    UART4_IRQHandler,       /* UART4 Rx and Tx                 */
    UART5_IRQHandler,       /* UART5 Rx and Tx                 */
    UART6_IRQHandler,       /* UART6 Rx and Tx                 */
    UART7_IRQHandler,       /* UART7 Rx and Tx                 */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    I2C2_IRQHandler,        /* I2C2 Master and Slave           */
    I2C3_IRQHandler,        /* I2C3 Master and Slave           */
    Timer4A_IRQHandler,     /* Timer 4 subtimer A              */
    Timer4B_IRQHandler,     /* Timer 4 subtimer B              */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    Timer5A_IRQHandler,     /* Timer 5 subtimer A              */
    Timer5B_IRQHandler,     /* Timer 5 subtimer B              */
    WideTimer0A_IRQHandler, /* Wide Timer 0 subtimer A         */
    WideTimer0B_IRQHandler, /* Wide Timer 0 subtimer B         */
    WideTimer1A_IRQHandler, /* Wide Timer 1 subtimer A         */
    WideTimer1B_IRQHandler, /* Wide Timer 1 subtimer B         */
    WideTimer2A_IRQHandler, /* Wide Timer 2 subtimer A         */
    WideTimer2B_IRQHandler, /* Wide Timer 2 subtimer B         */
    WideTimer3A_IRQHandler, /* Wide Timer 3 subtimer A         */
    WideTimer3B_IRQHandler, /* Wide Timer 3 subtimer B         */
    WideTimer4A_IRQHandler, /* Wide Timer 4 subtimer A         */
    WideTimer4B_IRQHandler, /* Wide Timer 4 subtimer B         */
    WideTimer5A_IRQHandler, /* Wide Timer 5 subtimer A         */
    WideTimer5B_IRQHandler, /* Wide Timer 5 subtimer B         */
    FPU_IRQHandler,         /* FPU                             */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    I2C4_IRQHandler,        /* I2C4 Master and Slave           */
    I2C5_IRQHandler,        /* I2C5 Master and Slave           */
    GPIOPortM_IRQHandler,   /* GPIO Port M                     */
    GPIOPortN_IRQHandler,   /* GPIO Port N                     */
    QEI2_IRQHandler,        /* Quadrature Encoder 2            */
    0,                            /* Reserved                        */
    0,                            /* Reserved                        */
    GPIOPortP0_IRQHandler,  /* GPIO Port P (Summary or P0)     */
    GPIOPortP1_IRQHandler,  /* GPIO Port P1                    */
    GPIOPortP2_IRQHandler,  /* GPIO Port P2                    */
    GPIOPortP3_IRQHandler,  /* GPIO Port P3                    */
    GPIOPortP4_IRQHandler,  /* GPIO Port P4                    */
    GPIOPortP5_IRQHandler,  /* GPIO Port P5                    */
    GPIOPortP6_IRQHandler,  /* GPIO Port P6                    */
    GPIOPortP7_IRQHandler,  /* GPIO Port P7                    */
    GPIOPortQ0_IRQHandler,  /* GPIO Port Q (Summary or Q0)     */
    GPIOPortQ1_IRQHandler,  /* GPIO Port Q1                    */
    GPIOPortQ2_IRQHandler,  /* GPIO Port Q2                    */
    GPIOPortQ3_IRQHandler,  /* GPIO Port Q3                    */
    GPIOPortQ4_IRQHandler,  /* GPIO Port Q4                    */
    GPIOPortQ5_IRQHandler,  /* GPIO Port Q5                    */
    GPIOPortQ6_IRQHandler,  /* GPIO Port Q6                    */
    GPIOPortQ7_IRQHandler,  /* GPIO Port Q7                    */
    GPIOPortR_IRQHandler,   /* GPIO Port R                     */
    GPIOPortS_IRQHandler,   /* GPIO Port S                     */
    PWM1Gen0_IRQHandler,    /* PWM 1 Generator 0               */
    PWM1Gen1_IRQHandler,    /* PWM 1 Generator 1               */
    PWM1Gen2_IRQHandler,    /* PWM 1 Generator 2               */
    PWM1Gen3_IRQHandler,    /* PWM 1 Generator 3               */
    PWM1Fault_IRQHandler,   /* PWM 1 Fault                     */
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern uint32_t __data_load__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void Reset_Handler(void)
{
    uint32_t *pui32Src, *pui32Dest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pui32Src = &__data_load__;
    for(pui32Dest = &__data_start__; pui32Dest < &__data_end__; )
    {
        *pui32Dest++ = *pui32Src++;
    }

    //
    // Zero fill the bss segment.
    //
    for (pui32Dest = &__bss_start__; pui32Dest < &__bss_end__; ++pui32Dest)
    {
        *pui32Dest = 0;
    }

    /*
    __asm("    ldr     r0, =__bss_start__\n"
          "    ldr     r1, =__bss_end__\n"
          "    mov     r2, #0\n"
          "    .thumb_func\n"
          "zero_loop:\n"
          "        cmp     r0, r1\n"
          "        it      lt\n"
          "        strlt   r2, [r0], #4\n"
          "        blt     zero_loop");
     */
    //
    // Enable the floating-point unit.  This must be done here to handle the
    // case where main() uses floating-point and the function prologue saves
    // floating-point registers (which will fault if floating-point is not
    // enabled).  Any configuration of the floating-point unit using DriverLib
    // APIs must be done here prior to the floating-point unit being enabled.
    //
    // Note that this does not use DriverLib since it might not be included in
    // this project.
    //
//    HWREG(0xE000ED88) = ((HWREG(0xE000ED88) & ~0x00F00000) | 0x00F00000);
    
    //
    // Call the application's entry point.
    //
    main();

    /* the previous code should not return, but assert just in case... */
    assert_failed("Reset_Handler", __LINE__);
}


/* fault exception handlers ------------------------------------------------*/
__attribute__((naked)) void NMI_Handler(void);
void NMI_Handler(void)
{
    __asm volatile (
        "    ldr r0,=str_nmi\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_nmi: .asciz \"NMI\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void MemManage_Handler(void);
void MemManage_Handler(void)
{
    __asm volatile (
        "    ldr r0,=str_mem\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_mem: .asciz \"MemManage\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void HardFault_Handler(void);
void HardFault_Handler(void)
{
    __asm volatile (
        "    ldr r0,=str_hrd\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_hrd: .asciz \"HardFault\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void BusFault_Handler(void);
void BusFault_Handler(void)
{
    __asm volatile (
        "    ldr r0,=str_bus\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_bus: .asciz \"BusFault\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void UsageFault_Handler(void);
void UsageFault_Handler(void)
{
    __asm volatile (
        "    ldr r0,=str_usage\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_usage: .asciz \"UsageFault\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void Default_Handler(void);
void Default_Handler(void)
{
    __asm volatile (
        "    ldr r0,=str_dflt\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_dflt: .asciz \"Default\"\n\t"
        "  .align 2\n\t"
    );
}
