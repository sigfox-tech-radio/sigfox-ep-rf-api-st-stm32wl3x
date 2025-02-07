# STM32WL3x RF API implementation example

## Description

This **STM32WL3x RF API** is a low level implementation example of the [Sigfox EP library](https://github.com/sigfox-tech-radio/sigfox-ep-lib), showing the `manuf/rf_api.c` file implementation for the [STM32WL3x](https://www.st.com/en/microcontrollers-microprocessors/stm32wl3x.html) SoC from ST-Microelectronics.

> [!WARNING]  
> The resulting radio performances of your device strongly depends on your **hardware design** (schematic, PCB routing, crystal oscillator placement, good RF practices, etc...). **Sigfox certification remains mandatory** whatever the software embedded in your device (including the Sigfox End-Point library and its implementation examples).

The table below shows the versions compatibility between this radio example and the Sigfox End-Point library.

| **STM32WL3X_RF_API** | **EP_LIB** |
|:---:|:---:|
| [v1.0](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x/releases/tag/v1.0) | >= [v4.0](https://github.com/sigfox-tech-radio/sigfox-ep-lib/releases/tag/v4.0) |

## Architecture

<p align="center">
<img src="https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x/wiki/images/sigfox_ep_rf_api_st_stm32wl3x_architecture.drawio.png" width="600"/>
</p>

## Compilation flags for optimization

This radio example inherits all the [Sigfox End-Point library flags](https://github.com/sigfox-tech-radio/sigfox-ep-lib/wiki/compilation-flags-for-optimization) and can be optimized accordingly.

The `SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE` flag must be enabled to use this example.

## How to add STM32WL3x RF API example to your project

### Dependencies

The **STM32WL3x RF API** is based on the [official STM32WL3x driver](https://github.com/STMicroelectronics/stm32wl3x-hal-driver) from ST-Microelectronics and also relies on **low level STM32WL3X_HW_API functions** (called board drivers) which need to be implemented to run on your specific hardware.

The templates are located in the `src/board` folder.

### Submodule

The best way to embed the STM32WL3x RF API example into your project is to use a [Git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules), in a similar way to the library. The radio driver will be seen as a sub-repository with independant history. It will be much easier to **upgrade the radio driver** or to **switch between versions** when necessary, by using the common `git pull` and `git checkout` commands within the `sigfox-ep-rf-api-st-stm32wl3x` folder.

To add the STM32WL3x RF API submodule, go to your project location and run the following commands:

```bash
mkdir lib
cd lib/
git submodule add https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x.git
```

This will clone the STM32WL3x RF API repository. At project level, you can commit the submodule creation with the following commands:

```bash
git commit --message "Add Sigfox STM32WL3x RF API submodule."
git push
```

With the submodule, you can easily:

* Update the radio driver to the **latest version**:

```bash
cd lib/sigfox-ep-rf-api-st-stm32wl3x/
git pull
git checkout master
```

* Use a **specific release**:

```bash
cd lib/sigfox-ep-rf-api-st-stm32wl3x/
git pull
git checkout <tag>
```

### Raw source code

You can [download](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x/releases) or clone any release of the STM32WL3x RF API example and copy all files into your project.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x.git
```

### Precompiled source code

You can [download](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x/releases) or clone any release of the STM32WL3x RF API example and copy all files into your project. If you do not plan to change your compilation flags in the future, you can perform a **precompilation step** before copying the file in your project. The precompilation will **remove all preprocessor directives** according to your flags selection, in order to produce a more **readable code**. Then you can copy the new files into your project.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x.git
```

To perform the precompilation, you have to install `cmake` and `unifdef` tools, and run the following commands:

```bash
cd sigfox-ep-rf-api-st-stm32wl3x/
mkdir build
cd build/

cmake -DSIGFOX_EP_LIB_DIR=<sigfox-ep-lib path> \
      -DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-arm-none-eabi.cmake" \
      -DTOOLCHAIN_PATH="<arm_toolchain_path>" \
      -DSIGFOX_EP_RC1_ZONE=ON \
      -DSIGFOX_EP_RC2_ZONE=ON \
      -DSIGFOX_EP_RC3_LBT_ZONE=ON \
      -DSIGFOX_EP_RC3_LDC_ZONE=ON \
      -DSIGFOX_EP_RC4_ZONE=ON \
      -DSIGFOX_EP_RC5_ZONE=ON \
      -DSIGFOX_EP_RC6_ZONE=ON \
      -DSIGFOX_EP_RC7_ZONE=ON \
      -DSIGFOX_EP_APPLICATION_MESSAGES=ON \
      -DSIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE=ON \
      -DSIGFOX_EP_BIDIRECTIONAL=ON \
      -DSIGFOX_EP_ASYNCHRONOUS=ON \
      -DSIGFOX_EP_LOW_LEVEL_OPEN_CLOSE=ON \
      -DSIGFOX_EP_REGULATORY=ON \
      -DSIGFOX_EP_LATENCY_COMPENSATION=ON \
      -DSIGFOX_EP_SINGLE_FRAME=ON \
      -DSIGFOX_EP_UL_BIT_RATE_BPS=OFF \
      -DSIGFOX_EP_TX_POWER_DBM_EIRP=OFF \
      -DSIGFOX_EP_T_IFU_MS=OFF \
      -DSIGFOX_EP_T_CONF_MS=OFF \
      -DSIGFOX_EP_UL_PAYLOAD_SIZE=OFF \
      -DSIGFOX_EP_AES_HW=ON \
      -DSIGFOX_EP_CRC_HW=OFF \
      -DSIGFOX_EP_MESSAGE_COUNTER_ROLLOVER=OFF \
      -DSIGFOX_EP_PARAMETERS_CHECK=ON \
      -DSIGFOX_EP_CERTIFICATION=ON \
      -DSIGFOX_EP_PUBLIC_KEY_CAPABLE=ON \
      -DSIGFOX_EP_VERBOSE=ON \
      -DSIGFOX_EP_ERROR_CODES=ON \
      -DSIGFOX_EP_ERROR_STACK=12 ..

make precompil_stm32wl3x_rf_api
```

The new files will be generated in the `build/precompil` folder.

### Static library

You can also [download](https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x/releases) or clone any release of the STM32WL3x RF API example and build a **static library**.

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-st-stm32wl3x.git
```

To build a static library, you have to install `cmake` tool and run the following commands:

```bash
cd sigfox-ep-rf-api-st-stm32wl3x/
mkdir build
cd build/

cmake -DSIGFOX_EP_LIB_DIR=<sigfox-ep-lib path> \
      -DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-arm-none-eabi.cmake" \
      -DTOOLCHAIN_PATH="<arm_toolchain_path>" \
      -DSIGFOX_EP_RC1_ZONE=ON \
      -DSIGFOX_EP_RC2_ZONE=ON \
      -DSIGFOX_EP_RC3_LBT_ZONE=ON \
      -DSIGFOX_EP_RC3_LDC_ZONE=ON \
      -DSIGFOX_EP_RC4_ZONE=ON \
      -DSIGFOX_EP_RC5_ZONE=ON \
      -DSIGFOX_EP_RC6_ZONE=ON \
      -DSIGFOX_EP_RC7_ZONE=ON \
      -DSIGFOX_EP_APPLICATION_MESSAGES=ON \
      -DSIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE=ON \
      -DSIGFOX_EP_BIDIRECTIONAL=ON \
      -DSIGFOX_EP_ASYNCHRONOUS=ON \
      -DSIGFOX_EP_LOW_LEVEL_OPEN_CLOSE=ON \
      -DSIGFOX_EP_REGULATORY=ON \
      -DSIGFOX_EP_LATENCY_COMPENSATION=ON \
      -DSIGFOX_EP_SINGLE_FRAME=ON \
      -DSIGFOX_EP_UL_BIT_RATE_BPS=OFF \
      -DSIGFOX_EP_TX_POWER_DBM_EIRP=OFF \
      -DSIGFOX_EP_T_IFU_MS=OFF \
      -DSIGFOX_EP_T_CONF_MS=OFF \
      -DSIGFOX_EP_UL_PAYLOAD_SIZE=OFF \
      -DSIGFOX_EP_AES_HW=ON \
      -DSIGFOX_EP_CRC_HW=OFF \
      -DSIGFOX_EP_MESSAGE_COUNTER_ROLLOVER=OFF \
      -DSIGFOX_EP_PARAMETERS_CHECK=ON \
      -DSIGFOX_EP_CERTIFICATION=ON \
      -DSIGFOX_EP_PUBLIC_KEY_CAPABLE=ON \
      -DSIGFOX_EP_VERBOSE=ON \
      -DSIGFOX_EP_ERROR_CODES=ON \
      -DSIGFOX_EP_ERROR_STACK=12 ..

make stm32wl3x_rf_api
```

The archive will be generated in the `build/lib` folder.
