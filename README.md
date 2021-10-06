# Just4Trionic-combi 

The project provide CAN and BDM functionality for Saab Trionic 8 ECU.

You can build the project with all supported [Mbed OS build tools](https://os.mbed.com/docs/mbed-os/latest/tools/index.html). However, this example project specifically refers to the command-line interface tool [Arm Mbed CLI](https://github.com/ARMmbed/mbed-cli#installing-mbed-cli).
(Note: To see a rendered example you can import into the Arm Online Compiler, please see our [import quick start](https://os.mbed.com/docs/mbed-os/latest/quick-start/online-with-the-online-compiler.html#importing-the-code).)

1. [Install Mbed CLI](https://os.mbed.com/docs/mbed-os/latest/quick-start/offline-with-mbed-cli.html).

1. Clone this repository on your system, and change the current directory to where the project was cloned:

    ```bash
    $ git clone git@github.com:witoldo7/just4tronic-combi.git && cd just4tronic-combi
    ```

## Application functionality

1. Just4Trionic CAN/BDM (Lawiacel can232)
2. CombiAdapter CAN/BDM 

## Building and running

1. Connect a USB cable between the USB port on the board and the host computer.
2. <a name="build_cmd"></a> Run the following command to build the example project and program the microcontroller flash memory:
    ```bash
    $ mbed compile -m <TARGET> -t <TOOLCHAIN> --flash
    ```
The binary is located at `./BUILD/<TARGET>/<TOOLCHAIN>/just4trionic-combi.bin`.

Alternatively, you can manually copy the binary to the board, which you mount on the host computer over USB.

Depending on the target, you can build the example project with the `GCC_ARM`, `ARM` or `IAR` toolchain. After installing Arm Mbed CLI, run the command below to determine which toolchain supports your target:

```bash
$ mbed compile -S
```

## Related Links

* [Just4Trionic](https://os.mbed.com/users/Just4pLeisure/code/Just4Trionic/).
* [CombiAdapter](https://www.trionictuning.com/forum/viewtopic.php?f=46&t=191).
* [CombiLib-mono](https://github.com/mattiasclaesson/combilib-mono).
* [Mbed boards](https://os.mbed.com/platforms/).