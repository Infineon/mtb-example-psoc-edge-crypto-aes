[Click here](../README.md) to view the README.

## Design and implementation

The design of this application is minimalistic to get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application Projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

<br>

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. Resource initialization for this example is performed by this CM33 non-secure project. It configures the system clocks, pins, clock to peripheral connections, and other platform resources. It then enables the CM55 core using the `Cy_SysEnableCM55()` function and the CM55 core is subsequently put to DeepSleep mode.

AES is a symmetric block cipher data encryption algorithm, which means that it uses the same key for encryption and decryption of data. The AES operation works on 128-bit block size and uses keys of 128 bits, 192 bits, or 256 bits of length.
This Code Example demonstates the use of the AES algorithm in ECB mode for both encryption and decryption.The implementation operates on fixed-size 16-byte blocks using a 128-bit symmetric key.
In this example, the user input message is read from the UART terminal and encrypted using the AES algorithm with a key length of 128 bits. Then, you can view the decrypted message on the UART terminal and verify that the decryption operation produces the same original encrypted message.

**Figure 2. Firmware flow chart**

![](images/flowchart.png)


<br />