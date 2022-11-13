set(STM32_HAL_SRC
    Src/stm32g4xx_hal.c
    Src/stm32g4xx_hal_adc.c
    Src/stm32g4xx_hal_adc_ex.c
    Src/stm32g4xx_hal_comp.c
    Src/stm32g4xx_hal_cordic.c
    Src/stm32g4xx_hal_cortex.c
    Src/stm32g4xx_hal_crc.c
    Src/stm32g4xx_hal_crc_ex.c
    Src/stm32g4xx_hal_dac.c
    Src/stm32g4xx_hal_dac_ex.c
    Src/stm32g4xx_hal_dma.c
    Src/stm32g4xx_hal_dma_ex.c
    Src/stm32g4xx_hal_exti.c
    Src/stm32g4xx_hal_fdcan.c
    Src/stm32g4xx_hal_flash.c
    Src/stm32g4xx_hal_flash_ex.c
    Src/stm32g4xx_hal_flash_ramfunc.c
    Src/stm32g4xx_hal_fmac.c
    Src/stm32g4xx_hal_gpio.c
    Src/stm32g4xx_hal_hrtim.c
    Src/stm32g4xx_hal_i2c.c
    Src/stm32g4xx_hal_i2c_ex.c
    Src/stm32g4xx_hal_msp_template.c
    Src/stm32g4xx_hal_opamp.c
    Src/stm32g4xx_hal_opamp_ex.c
    Src/stm32g4xx_hal_pwr.c
    Src/stm32g4xx_hal_pwr_ex.c
    Src/stm32g4xx_hal_rcc.c
    Src/stm32g4xx_hal_rcc_ex.c
    Src/stm32g4xx_hal_smbus.c
    Src/stm32g4xx_hal_spi.c
    Src/stm32g4xx_hal_spi_ex.c
    Src/stm32g4xx_hal_sram.c
    Src/stm32g4xx_hal_tim.c
    Src/stm32g4xx_hal_tim_ex.c
    Src/stm32g4xx_hal_timebase_tim_template.c
    Src/stm32g4xx_hal_uart.c
    Src/stm32g4xx_hal_uart_ex.c
    Src/stm32g4xx_hal_usart.c
    Src/stm32g4xx_hal_usart_ex.c
    Src/stm32g4xx_hal_wwdg.c
    system_stm32g4xx.c
    startup_stm32g431xx.s
)

set(STM32_HAL_EXT_SRC
    Src/stm32g4xx_hal_i2s.c
    Src/stm32g4xx_hal_irda.c
    Src/stm32g4xx_hal_iwdg.c
    Src/stm32g4xx_hal_lptim.c
    Src/stm32g4xx_hal_sai.c
    Src/stm32g4xx_hal_sai_ex.c
    Src/stm32g4xx_hal_smartcard.c
    Src/stm32g4xx_hal_smartcard_ex.c
    Src/stm32g4xx_hal_nand.c
    Src/stm32g4xx_hal_nor.c
    Src/stm32g4xx_hal_pcd.c
    Src/stm32g4xx_hal_pcd_ex.c
    Src/stm32g4xx_hal_qspi.c
    Src/stm32g4xx_hal_rtc.c
    Src/stm32g4xx_hal_rtc_ex.c
    Src/stm32g4xx_hal_cryp.c
    Src/stm32g4xx_hal_cryp_ex.c
    Src/stm32g4xx_hal_rng.c
)

set(STM32_LL_SRC
    Src/stm32g4xx_ll_adc.c
    Src/stm32g4xx_ll_comp.c
    Src/stm32g4xx_ll_cordic.c
    Src/stm32g4xx_ll_crc.c
    Src/stm32g4xx_ll_crs.c
    Src/stm32g4xx_ll_dac.c
    Src/stm32g4xx_ll_dma.c
    Src/stm32g4xx_ll_exti.c
    Src/stm32g4xx_ll_fmac.c
    Src/stm32g4xx_ll_fmc.c
    Src/stm32g4xx_ll_gpio.c
    Src/stm32g4xx_ll_hrtim.c
    Src/stm32g4xx_ll_i2c.c
    Src/stm32g4xx_ll_lptim.c
    Src/stm32g4xx_ll_lpuart.c
    Src/stm32g4xx_ll_opamp.c
    Src/stm32g4xx_ll_pwr.c
    Src/stm32g4xx_ll_rcc.c
    Src/stm32g4xx_ll_rng.c
    Src/stm32g4xx_ll_rtc.c
    Src/stm32g4xx_ll_spi.c
    Src/stm32g4xx_ll_tim.c
    Src/stm32g4xx_ll_ucpd.c
    Src/stm32g4xx_ll_usart.c
    Src/stm32g4xx_ll_usb.c
    Src/stm32g4xx_ll_utils.c
)

if(USE_HAL_LIB)
    set(HAL_SRC ${STM32_HAL_SRC})
else()
    set(HAL_SRC ${STM32_LL_SRC})
endif()

add_library(hal STATIC ${HAL_SRC})

if(USE_LL_LIB)
    target_compile_definitions(hal PUBLIC -DUSE_FULL_LL_DRIVER)
endif()

if(USE_HAL_LIB)
    target_compile_definitions(hal PUBLIC -DUSE_HAL_DRIVER)
endif()

target_link_libraries(hal PUBLIC cmsis)
target_include_directories(hal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
    PRIVATE Src)
