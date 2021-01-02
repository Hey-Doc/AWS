################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api.c \
D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api_calibration.c \
D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api_core.c \
D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api_ranging.c \
D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api_strings.c \
D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_platform_log.c 

OBJS += \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api.o \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api_calibration.o \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api_core.o \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api_ranging.o \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api_strings.o \
./Drivers/BSP/Components/vl53l0x/vl53l0x_platform_log.o 

C_DEPS += \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api.d \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api_calibration.d \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api_core.d \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api_ranging.d \
./Drivers/BSP/Components/vl53l0x/vl53l0x_api_strings.d \
./Drivers/BSP/Components/vl53l0x/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/vl53l0x/vl53l0x_api.o: D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_WIFI -DSENSOR '-DMBEDTLS_USER_CONFIG_FILE=<mbedtls_user_config.h>' -DENABLE_IMAGE_STATE_HANDLING -DDEBUG -DSTM32L4S5xx '-D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__' -c -I../../Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/wifi/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/posix/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/tinycbor -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/network_manager -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/include -I../../../../../../../Middlewares/ST/STM32_Key_Management_Services/tKMS -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/secure_sockets/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/mbedtls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/http_parser -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include/private -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/portable/GCC/ARM_CM4F -I../../../../BootLoader_STSAFE/2_Images_SBSFU/SBSFU/App -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/jsmn -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/pkcs11/include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/serializer/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/shadow/include -I../../../../BootLoader_STSAFE/2_Images_SECoreBin/Inc -I../../../../BootLoader_STSAFE/Linker_Common/STM32CubeIDE -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/https/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/freertos/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/dev_mode_key_provisioning/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/include -I../../config_files -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/mqtt/include -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/tls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/mbedtls -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/utils/include -I../../../../../../../Drivers/stm32l4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/include -I../../../../../../../Drivers/BSPv2/Components/es_wifi -I../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/defender/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/crypto/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/pkcs11 -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_posix/include -I../../../../../../../Drivers/BSPv2/B-L475E-IOT01 -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm6dsl -I../../../../../../../Drivers/BSP/Components/vl53l0x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/vl53l0x/vl53l0x_api.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/Components/vl53l0x/vl53l0x_api_calibration.o: D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api_calibration.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_WIFI -DSENSOR '-DMBEDTLS_USER_CONFIG_FILE=<mbedtls_user_config.h>' -DENABLE_IMAGE_STATE_HANDLING -DDEBUG -DSTM32L4S5xx '-D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__' -c -I../../Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/wifi/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/posix/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/tinycbor -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/network_manager -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/include -I../../../../../../../Middlewares/ST/STM32_Key_Management_Services/tKMS -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/secure_sockets/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/mbedtls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/http_parser -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include/private -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/portable/GCC/ARM_CM4F -I../../../../BootLoader_STSAFE/2_Images_SBSFU/SBSFU/App -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/jsmn -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/pkcs11/include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/serializer/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/shadow/include -I../../../../BootLoader_STSAFE/2_Images_SECoreBin/Inc -I../../../../BootLoader_STSAFE/Linker_Common/STM32CubeIDE -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/https/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/freertos/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/dev_mode_key_provisioning/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/include -I../../config_files -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/mqtt/include -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/tls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/mbedtls -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/utils/include -I../../../../../../../Drivers/stm32l4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/include -I../../../../../../../Drivers/BSPv2/Components/es_wifi -I../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/defender/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/crypto/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/pkcs11 -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_posix/include -I../../../../../../../Drivers/BSPv2/B-L475E-IOT01 -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm6dsl -I../../../../../../../Drivers/BSP/Components/vl53l0x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/vl53l0x/vl53l0x_api_calibration.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/Components/vl53l0x/vl53l0x_api_core.o: D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api_core.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_WIFI -DSENSOR '-DMBEDTLS_USER_CONFIG_FILE=<mbedtls_user_config.h>' -DENABLE_IMAGE_STATE_HANDLING -DDEBUG -DSTM32L4S5xx '-D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__' -c -I../../Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/wifi/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/posix/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/tinycbor -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/network_manager -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/include -I../../../../../../../Middlewares/ST/STM32_Key_Management_Services/tKMS -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/secure_sockets/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/mbedtls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/http_parser -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include/private -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/portable/GCC/ARM_CM4F -I../../../../BootLoader_STSAFE/2_Images_SBSFU/SBSFU/App -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/jsmn -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/pkcs11/include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/serializer/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/shadow/include -I../../../../BootLoader_STSAFE/2_Images_SECoreBin/Inc -I../../../../BootLoader_STSAFE/Linker_Common/STM32CubeIDE -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/https/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/freertos/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/dev_mode_key_provisioning/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/include -I../../config_files -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/mqtt/include -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/tls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/mbedtls -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/utils/include -I../../../../../../../Drivers/stm32l4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/include -I../../../../../../../Drivers/BSPv2/Components/es_wifi -I../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/defender/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/crypto/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/pkcs11 -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_posix/include -I../../../../../../../Drivers/BSPv2/B-L475E-IOT01 -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm6dsl -I../../../../../../../Drivers/BSP/Components/vl53l0x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/vl53l0x/vl53l0x_api_core.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/Components/vl53l0x/vl53l0x_api_ranging.o: D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api_ranging.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_WIFI -DSENSOR '-DMBEDTLS_USER_CONFIG_FILE=<mbedtls_user_config.h>' -DENABLE_IMAGE_STATE_HANDLING -DDEBUG -DSTM32L4S5xx '-D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__' -c -I../../Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/wifi/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/posix/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/tinycbor -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/network_manager -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/include -I../../../../../../../Middlewares/ST/STM32_Key_Management_Services/tKMS -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/secure_sockets/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/mbedtls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/http_parser -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include/private -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/portable/GCC/ARM_CM4F -I../../../../BootLoader_STSAFE/2_Images_SBSFU/SBSFU/App -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/jsmn -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/pkcs11/include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/serializer/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/shadow/include -I../../../../BootLoader_STSAFE/2_Images_SECoreBin/Inc -I../../../../BootLoader_STSAFE/Linker_Common/STM32CubeIDE -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/https/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/freertos/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/dev_mode_key_provisioning/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/include -I../../config_files -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/mqtt/include -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/tls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/mbedtls -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/utils/include -I../../../../../../../Drivers/stm32l4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/include -I../../../../../../../Drivers/BSPv2/Components/es_wifi -I../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/defender/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/crypto/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/pkcs11 -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_posix/include -I../../../../../../../Drivers/BSPv2/B-L475E-IOT01 -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm6dsl -I../../../../../../../Drivers/BSP/Components/vl53l0x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/vl53l0x/vl53l0x_api_ranging.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/Components/vl53l0x/vl53l0x_api_strings.o: D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_api_strings.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_WIFI -DSENSOR '-DMBEDTLS_USER_CONFIG_FILE=<mbedtls_user_config.h>' -DENABLE_IMAGE_STATE_HANDLING -DDEBUG -DSTM32L4S5xx '-D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__' -c -I../../Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/wifi/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/posix/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/tinycbor -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/network_manager -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/include -I../../../../../../../Middlewares/ST/STM32_Key_Management_Services/tKMS -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/secure_sockets/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/mbedtls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/http_parser -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include/private -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/portable/GCC/ARM_CM4F -I../../../../BootLoader_STSAFE/2_Images_SBSFU/SBSFU/App -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/jsmn -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/pkcs11/include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/serializer/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/shadow/include -I../../../../BootLoader_STSAFE/2_Images_SECoreBin/Inc -I../../../../BootLoader_STSAFE/Linker_Common/STM32CubeIDE -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/https/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/freertos/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/dev_mode_key_provisioning/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/include -I../../config_files -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/mqtt/include -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/tls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/mbedtls -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/utils/include -I../../../../../../../Drivers/stm32l4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/include -I../../../../../../../Drivers/BSPv2/Components/es_wifi -I../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/defender/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/crypto/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/pkcs11 -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_posix/include -I../../../../../../../Drivers/BSPv2/B-L475E-IOT01 -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm6dsl -I../../../../../../../Drivers/BSP/Components/vl53l0x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/vl53l0x/vl53l0x_api_strings.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/Components/vl53l0x/vl53l0x_platform_log.o: D:/A/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/BSP/Components/vl53l0x/vl53l0x_platform_log.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_WIFI -DSENSOR '-DMBEDTLS_USER_CONFIG_FILE=<mbedtls_user_config.h>' -DENABLE_IMAGE_STATE_HANDLING -DDEBUG -DSTM32L4S5xx '-D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__' -c -I../../Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/wifi/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/posix/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/tinycbor -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/network_manager -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/include -I../../../../../../../Middlewares/ST/STM32_Key_Management_Services/tKMS -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/secure_sockets/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/mbedtls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/http_parser -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include/private -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/portable/GCC/ARM_CM4F -I../../../../BootLoader_STSAFE/2_Images_SBSFU/SBSFU/App -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/jsmn -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/pkcs11/include -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/serializer/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/shadow/include -I../../../../BootLoader_STSAFE/2_Images_SECoreBin/Inc -I../../../../BootLoader_STSAFE/Linker_Common/STM32CubeIDE -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/ota/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/https/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/freertos/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/freertos_kernel/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/demos/dev_mode_key_provisioning/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/include -I../../config_files -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/aws/greengrass/src -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/mqtt/include -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/standard/common/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/tls/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/pkcs11/mbedtls -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/utils/include -I../../../../../../../Drivers/stm32l4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/abstractions/platform/include -I../../../../../../../Drivers/BSPv2/Components/es_wifi -I../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/c_sdk/aws/defender/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/crypto/include -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/3rdparty/pkcs11 -I../../../../../../../Middlewares/Third_Party/amazon-freertos/libraries/freertos_plus/standard/freertos_plus_posix/include -I../../../../../../../Drivers/BSPv2/B-L475E-IOT01 -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../Drivers/BSP/Components/lps22hb -I../../../../../../../Drivers/BSP/Components/lsm6dsl -I../../../../../../../Drivers/BSP/Components/vl53l0x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/vl53l0x/vl53l0x_platform_log.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

