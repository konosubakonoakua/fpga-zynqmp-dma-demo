<p>
<div align="center">
  <h1 align="center">User space mappable DMA buffer</h1>
</div>
</p>

<p align="center">
  <img src="./Images/petalinux-uio.png" width="100">
</p>

# Overview
The UIO Framework includes a UIO DMA driver named `uio-dmem-genirq.c`, which can be enabled with the `UIO_DMEM_GENIRQ` kernel configuration flag. This DMA driver allocates DMA buffers outside of the Linux memory management system, allowing access through `/dev/mem` using `mmap()`. Unfortunately, because Linux disables the CPU cache in this non-managed area, performance is significantly slow.

An alternative to the UIO DMA driver `uio-dmem-genirq.c` is an open-source driver called <a href="https://github.com/ikwzm/udmabuf">u-dma-buf</a>. The `udmabuf` driver allocates buffers in the Contiguous Memory Allocator (<a href="https://developer.toradex.com/software/linux-resources/linux-features/contiguous-memory-allocator-cma-linux/">CMA</a>) area of the Linux kernel, which allows Linux to enable the <a href="https://www.howtogeek.com/854138/what-is-cpu-cache/">CPU cache</a> for improved performance.

# Table Of Contents
- [Overview](#overview)
- [Introduction To U-DMA-BUF](#introduction-to-u-dma-buf)
- [Architecture Of U-DMA-BUF](#architecture-of-u-dma-buf)
- [U-DMA-BUF Example Design](#-u-dma-buf--example-design)
- [Vivado Internals](#vivado-internals)
- [Minimal working hardware](#minimal-working-hardware)
- [Vivado Design](#vivado-design)
- [Clocks](#clocks)
- [Minimal working software](#minimal-working-software)
- [Petalinux Build](#petalinux-build)
  * [Create/Build UIO Application](#create-build-uio-application)
  * [Create/Build UDMABUF Driver](#create-build-udmabuf-driver)
  * [Update Device Tree](#update-device-tree)
  * [Build New Images](#build-new-images)
  * [Verify Driver And Application](#verify-driver-and-application)
  * [Successful Run](#successful-run)
- [References](#references)


# Introduction To U-DMA-BUF
`u-dma-buf` is a Linux device driver that allocates contiguous memory blocks in the kernel space as DMA buffers and makes them available from the user space. It is intended that these memory blocks are used as DMA buffers when a user application implements device driver in user space using UIO (`User space I/O`).

A DMA buffer allocated by u-dma-buf can be accessed from the user space by opening the device file (e.g. `/dev/udmabuf-rx`) and mapping to the user memory space, or using the `read()/write()` functions.

CPU cache for the allocated DMA buffer can be disabled by setting the **O_SYNC** flag when opening the device file. It is also possible to flush or invalidate CPU cache while retaining CPU cache enabled.

The physical address of a DMA buffer allocated by u-dma-buf can be obtained by reading `/sys/class/u-dma-buf/udmabuf-rx/phys_addr`.

The size of a DMA buffer and the device minor number can be specified in the device tree.

# Architecture Of U-DMA-BUF
<p align="left">
  <img src="./Images/udmabuf_arch.png" width="800">
</p>

# U-DMA-BUF Example Design
This GitLab demo is a port of an antiquated u-dma-buf example design from 2012, originally developed by Lauri Vosandi. It uses the u-dma-drv driver created by <a href="https://github.com/ikwzm">Iichiro Kawazome</a>. The original example was based on a Zynq processor, utilizing 2015 Xilinx tools, and used device tree overlays. This port was created for the ZynqMP Processor on the Avnet ZCU1CG demo board using AMD/Xilinx 2023.2 development tools, with the FPGA manager disabled.

# Vivado Internals
AXI DMA distinguishes two channels: MM2S (memory-mapped to stream) transports data from DDR memory to FPGA and S2MM (stream to memory-mapped) transports arbitrary data stream to DDR memory (see <a href="https://docs.amd.com/r/en-US/pg021_axi_dma/Core-Overview">AXI DMA LogiCORE IP Product Guide (PG021)</a>).

<p align="left">
  <img src="./Images/axi_dma_internals.png" width="600">
</p>

# Minimal working hardware
The simplest way to instantiate AXI DMA on ZynqMP SOC is to take a reference board design (ex.ZCU1CG), strip unnecessary components, add AXI Direct Memory Access IP-core and connect the output stream port to it's input stream port. This essentially implements memcpy functionality which can be triggered from ARM core but offloaded to programmable fabric.

<p align="left">
  <img src="./Images/axi_block_diagram.png" width="500">
</p>

# Vivado Design
The high level block design is shown below. High speed clock line is highlighted in orange as it runs on higher frequency of 150MHz while the general purpose port runs at 100MHz. Clock domain errors can usually be tracked back to conflicting clock lines. This is further explained in the end of this article (**Note** - the vivado design files (tcl and xsa) can be downloaded from the `Files` directory at the lop of this lab).
<p align="left">
  <img src="./Images/vivado_block_design.png" width="800">
</p>

In the AXI Direct Memory Access IP-core customization dialog read channel and write channel correspond respectively to `MM2S` and `S2MM` portions of the DMA block. Memory map data width of 32 bits means that 4 bytes will be transferred during one bus cycle. This means the tdata port of the stream interface will be 32 bits wide.

<p align="left">
  <img src="./Images/axi_direct_memory_access.png" width="800">
</p>

AXI Direct Memory Access component's control register, status register and transfer address registers are accessible via the AXI Lite slave port which is memory mapped to address range of `0xA0000000 - 0xA000FFFF`. The whole memory range of `0x00000000-0x3FFFFFFF` is accessible via both stream to memory-mapped and memory-mapped to stream channel. AXI DMA 1 documentation has the offsets of the registers accessible via AXI Lite port. In this case MM2S control register of 32-bits is accessible at `0xA0000000`, MM2S status register of 32-bits at `0xA0000004` and so forth.

<p align="left">
  <img src="./Images/address_mapping.png" width="800">
</p>

**Note:** customizing the AXI Direct Memory Access IP-core parameters causes memory ranges to be reset under Address Editor!

# Clocks
High-speed slave port (`S_AXI_HP0_FPD`) and associated ports (`M00_AXI, S00_AXI, S01_AXI, M_AXI_MM2S, M_AXI_S2MM`) run at 150MHz dictated by `PL_CLK1`. Master in this case means that the bus transfers are initiated by the master which in this case is the AXI Direct Memory Access component. AXI Interconnect 0 in this case is acting merely as a switch in an ethernet network multiplexing multiple AXI ports (`S00_AXI, S01_AXI`) to single M00_AXI.

General-purpose port (`M_AXI_HPM0_FPD`), including all AXI Lite slaves ,run at 100MHz. In this case ZynqMP Processing System is the transfer initiator. AXI Interconnect 1, similarly to AXI Interconnect 0, allows access to multiple AXI Lite slaves (`S_AXI_LITE` in this case) via single AXI Lite master port (`M_AXI_HPM0_FPD`) on the zynqmp_fsbl Processing System.
<p align="left">
  <img src="./Images/clock_config.png" width="800">
</p>

# Minimal working software
Physical memory can be accessed in Linux via `/dev/mem` block device. This makes it possible to access AXI Lite registers simply by reading/writing to a memory mapped range from `/dev/mem`. To use DMA component minimally four steps have to be taken:

1. Start the DMA channel (MM2S, S2MM or both) by writing 1 to control register

1. Write start/destination addresses to corresponding registers

1. To initiate the transfer(s) write transfer length(s) to corresponding register(s).

1. (**Optional**) Monitor status register for IOC_Irq flag.

In this case we're copying 32 bytes from the TX buffer physical address of **0x3625000** to the RX buffer physical address of **0x36254000**, determined below.
```ruby
root@TARGET> dmesg |grep cma|grep Reserved
 [    0.000000] cma: Reserved 64 MiB at 0x0000000036200000
root@TARGET> cat /sys/class/u-dma-buf/udmabuf-tx/phys_addr
0x0000000036250000
root@TARGET> cat /sys/class/u-dma-buf/udmabuf-rx/phys_addr
0x0000000036254000
```

**Note** that kernel may allocate memory for other processes in that range and that is the primary reason to write a kernel module which would `request_mem_region` so no other processes would overlap with the memory range. Besides reserving memory ranges the kernel module provides a sanitized way of accessing the hardware from userspace applications via `/dev/uio*` block devices.
```ruby
/**
 * Proof of concept offloaded memcopy using AXI Direct Memory Access v7.1
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/mman.h>

#define MM2S_CONTROL_REGISTER 0x00
#define MM2S_STATUS_REGISTER 0x04
#define MM2S_START_ADDRESS 0x18
#define MM2S_LENGTH 0x28

#define S2MM_CONTROL_REGISTER 0x30
#define S2MM_STATUS_REGISTER 0x34
#define S2MM_DESTINATION_ADDRESS 0x48
#define S2MM_LENGTH 0x58

unsigned int dma_set(unsigned int* dma_virtual_address, int offset, unsigned int value) {
    dma_virtual_address[offset>>2] = value;
}

unsigned int dma_get(unsigned int* dma_virtual_address, int offset) {
    return dma_virtual_address[offset>>2];
}

int dma_mm2s_sync(unsigned int* dma_virtual_address) {
    unsigned int mm2s_status =  dma_get(dma_virtual_address, MM2S_STATUS_REGISTER);
    while(!(mm2s_status & 1<<12) || !(mm2s_status & 1<<1) ){
        dma_s2mm_status(dma_virtual_address);
        dma_mm2s_status(dma_virtual_address);

        mm2s_status =  dma_get(dma_virtual_address, MM2S_STATUS_REGISTER);
    }
}

int dma_s2mm_sync(unsigned int* dma_virtual_address) {
    unsigned int s2mm_status = dma_get(dma_virtual_address, S2MM_STATUS_REGISTER);
    while(!(s2mm_status & 1<<12) || !(s2mm_status & 1<<1)){
        dma_s2mm_status(dma_virtual_address);
        dma_mm2s_status(dma_virtual_address);

        s2mm_status = dma_get(dma_virtual_address, S2MM_STATUS_REGISTER);
    }
}

void dma_s2mm_status(unsigned int* dma_virtual_address) {
    unsigned int status = dma_get(dma_virtual_address, S2MM_STATUS_REGISTER);
    printf("Stream to memory-mapped status (0x%08x@0x%02x):", status, S2MM_STATUS_REGISTER);
    if (status & 0x00000001) printf(" halted"); else printf(" running");
    if (status & 0x00000002) printf(" idle");
    if (status & 0x00000008) printf(" SGIncld");
    if (status & 0x00000010) printf(" DMAIntErr");
    if (status & 0x00000020) printf(" DMASlvErr");
    if (status & 0x00000040) printf(" DMADecErr");
    if (status & 0x00000100) printf(" SGIntErr");
    if (status & 0x00000200) printf(" SGSlvErr");
    if (status & 0x00000400) printf(" SGDecErr");
    if (status & 0x00001000) printf(" IOC_Irq");
    if (status & 0x00002000) printf(" Dly_Irq");
    if (status & 0x00004000) printf(" Err_Irq");
    printf("\n");
}

void dma_mm2s_status(unsigned int* dma_virtual_address) {
    unsigned int status = dma_get(dma_virtual_address, MM2S_STATUS_REGISTER);
    printf("Memory-mapped to stream status (0x%08x@0x%02x):", status, MM2S_STATUS_REGISTER);
    if (status & 0x00000001) printf(" halted"); else printf(" running");
    if (status & 0x00000002) printf(" idle");
    if (status & 0x00000008) printf(" SGIncld");
    if (status & 0x00000010) printf(" DMAIntErr");
    if (status & 0x00000020) printf(" DMASlvErr");
    if (status & 0x00000040) printf(" DMADecErr");
    if (status & 0x00000100) printf(" SGIntErr");
    if (status & 0x00000200) printf(" SGSlvErr");
    if (status & 0x00000400) printf(" SGDecErr");
    if (status & 0x00001000) printf(" IOC_Irq");
    if (status & 0x00002000) printf(" Dly_Irq");
    if (status & 0x00004000) printf(" Err_Irq");
    printf("\n");
}

void memdump(void* virtual_address, int byte_count) {
    char *p = virtual_address;
    int offset;
    for (offset = 0; offset < byte_count; offset++) {
        printf("%02x", p[offset]);
        if (offset % 4 == 3) { printf(" "); }
    }
    printf("\n");
}

int main() {
  int            uio_fd;
  unsigned int*  virtual_address;
  struct udmabuf src_buf;
  struct udmabuf dst_buf;
  unsigned int*  virtual_source_address;
  unsigned int*  virtual_destination_address;
  unsigned int   test_size = 32;
  int            buf_cache_on = 1;

  /* Open dma device tree node (ex. dma@a0000000) */
  if ((uio_fd = uio_open("dma")) == -1) {
      printf("Can not open loopback_dma uio\n");
      exit(1);
  }

  virtual_address = (unsigned int*)mmap(NULL, 65535, PROT_READ|PROT_WRITE, MAP_SHARED, uio_fd, 0);

  if (udmabuf_open(&src_buf, "udmabuf-tx", buf_cache_on) == -1) {
      printf("Can not open /dev/udmabuf-tx\n");
      exit(1);
  }
  virtual_source_address = (unsigned int*)src_buf.buf;
  udmabuf_sync_area(&src_buf, 0, test_size, 1);

  if (udmabuf_open(&dst_buf, "udmabuf-rx", buf_cache_on) == -1) {
      printf("Can not open /dev/udmabuf-rx\n");
      exit(1);
  }
  virtual_destination_address = (unsigned int*)dst_buf.buf;
  udmabuf_sync_area(&dst_buf, 0, test_size, 2);

  virtual_source_address[0]= 0x11223344; // Write random stuff to source block
  memset(virtual_destination_address, 0, test_size); // Clear destination block

  printf("Source memory block:      "); memdump(virtual_source_address, test_size);
  printf("Destination memory block: "); memdump(virtual_destination_address, test_size);

  if (buf_cache_on == 1) {
      udmabuf_sync_for_device(&src_buf);
      udmabuf_sync_for_device(&dst_buf);
  }

  printf("Resetting DMA\n");
  dma_set(virtual_address, S2MM_CONTROL_REGISTER, 4);
  dma_set(virtual_address, MM2S_CONTROL_REGISTER, 4);
  dma_s2mm_status(virtual_address);
  dma_mm2s_status(virtual_address);

  printf("Halting DMA\n");
  dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0);
  dma_set(virtual_address, MM2S_CONTROL_REGISTER, 0);
  dma_s2mm_status(virtual_address);
  dma_mm2s_status(virtual_address);

  printf("Writing destination address\n");
  dma_set(virtual_address, S2MM_DESTINATION_ADDRESS, (unsigned int)dst_buf.phys_addr); // Write destination address
  dma_s2mm_status(virtual_address);

  printf("Writing source address...\n");
  dma_set(virtual_address, MM2S_START_ADDRESS, (unsigned int)src_buf.phys_addr); // Write source address
  dma_mm2s_status(virtual_address);

  printf("Starting S2MM channel with all interrupts masked...\n");
  dma_set(virtual_address, S2MM_CONTROL_REGISTER, 0xf001);
  dma_s2mm_status(virtual_address);

  printf("Starting MM2S channel with all interrupts masked...\n");
  dma_set(virtual_address, MM2S_CONTROL_REGISTER, 0xf001);
  dma_mm2s_status(virtual_address);

  printf("Writing S2MM transfer length...\n");
  dma_set(virtual_address, S2MM_LENGTH, test_size);
  dma_s2mm_status(virtual_address);

  printf("Writing MM2S transfer length...\n");
  dma_set(virtual_address, MM2S_LENGTH, test_size);
  dma_mm2s_status(virtual_address);

  printf("Waiting for MM2S synchronization...\n");
  dma_mm2s_sync(virtual_address);

  printf("Waiting for S2MM sychronization...\n");
  dma_s2mm_sync(virtual_address); // If this locks up make sure all memory ranges are assigned under Address Editor!

  dma_s2mm_status(virtual_address);
  dma_mm2s_status(virtual_address);

  if (buf_cache_on == 1) {
      udmabuf_sync_for_cpu(&src_buf);
      udmabuf_sync_for_cpu(&dst_buf);
  }

  printf("Destination memory block: "); memdump(virtual_destination_address, test_size);

  udmabuf_close(&src_buf);
  udmabuf_close(&dst_buf);
  close(uio_fd);    
}
```
# Petalinux Build
Begin by downloading the lab files from the **Files** directory at the top of this lab.

## Create/Build UIO Application
```ruby
HOST> cd /YourPetalinuxProject
HOST> petalinux-create -t apps -n loopback-dma-test --enable
HOST> cd project-spec/meta-user/recipes-apps/loopback-dma-test/files
HOST> cp ~/Downloads/loop-dma-test.c .
HOST> petalinux-build -c loop-dma-test
```
## Create/Build UDMABUF Driver
```ruby
HOST> petalinux-create -t modules -n u-dma-buf --enable
HOST> cd project-spec/meta-user/recipes-modules/u-dma-buf/files
HOST> cp ~/Downloads/u-dma-buf.c .
HOST> petalinux-build -c u-dma-buf
```
## Update Device Tree
The following device tree modifications a required to support the u-dma-buf driver. See the <a href="https://github.com/ikwzm/udmabuf/tree/master">u-dma-buf</a> driver repository for a full list of node properties that can be added to the device tree.

The `size` of a DMA buffer and the `device minor number` can be specified when the device driver is loaded. The `sync-direction` property is used to set the direction of DMA when manually controlling the cache of u-dma-buf.

**Note** - The `memory-region` property is optional. When the memory-region property is not specified, u-dma-buf allocates the DMA buffer from the CMA area allocated to the Linux kernel.

```ruby
HOST> cd /YourPetalinuxProject
HOST> cd /project-spec/meta-user/recipes-bsp/device-tree/files
HOST> cp ~/Downloads/system-user.dtsi .
HOST> petalinux-build -c device-tree
```

```ruby
/include/ "system-conf.dtsi"
/ {

        chosen {
              bootargs = "earlycon console=ttyPS0,115200 clk_ignore_unused uio_pdrv_genirq.of_id=generic-uio,ui_pdrv root=/dev/ram0 rw";
        };

        udmabuf-tx {
              compatible  = "ikwzm,u-dma-buf";
              device-name = "udmabuf-tx";
              size = <0x4000>;
              sync-direction = <1>;
        };

        udmabuf-rx {
              compatible  = "ikwzm,u-dma-buf";
              device-name = "udmabuf-rx";
              size = <0x4000>;
              sync-direction = <2>;
        };
};

&axi_dma_0 {
        compatible = "generic-uio,ui_pdrv";
};
```
## Build New Images
```ruby
HOST> petalinux-build -x distclean; petalinux-build
```

## Verify Driver And Application
```ruby
root@TARGET> ls /dev/udma*
/dev/udmabuf     /dev/udmabuf-rx  /dev/udmabuf-tx

root@TARGET> ls /sys/class/u-dma-buf/*
/sys/class/u-dma-buf/udmabuf-rx:
debug_vma        dma_coherent     power            subsystem        sync_for_device  sync_owner
dev              driver_version   quirk_mmap_mode  sync_direction   sync_mode        sync_size
device           phys_addr        size             sync_for_cpu     sync_offset      uevent

/sys/class/u-dma-buf/udmabuf-tx:
debug_vma        dma_coherent     power            subsystem        sync_for_device  sync_owner
dev              driver_version   quirk_mmap_mode  sync_direction   sync_mode        sync_size
device           phys_addr        size             sync_for_cpu     sync_offset      uevent

root@TARGET> lsmod
Module                  Size  Used by
u_dma_buf              28672  0
cfg80211              360448  0
uio_pdrv_genirq        16384  0

root@TARGET> which loopback-dma-test
/usr/bin/loopback-dma-test
```

## Successful Run
```ruby
root@TARGET> loopback-dma-test
scan /sys/class/uio/./name
scan /sys/class/uio/../name
scan /sys/class/uio/uio1/name
read => axi-pmon

scan /sys/class/uio/uio4/name
read => dma

found dma in /dev/uio4
Source memory block:      44332211 00000000 00000000 00000000 00000000 00000000 00000000 00000000
Destination memory block: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
Resetting DMA
Stream to memory-mapped status (0x00000001@0x34): halted
Memory-mapped to stream status (0x00000001@0x04): halted
Halting DMA
Stream to memory-mapped status (0x00000001@0x34): halted
Memory-mapped to stream status (0x00000001@0x04): halted
Writing destination address
Stream to memory-mapped status (0x00000001@0x34): halted
Writing source address...
Memory-mapped to stream status (0x00000001@0x04): halted
Starting S2MM channel with all interrupts masked...
Stream to memory-mapped status (0x00000000@0x34): running
Starting MM2S channel with all interrupts masked...
Memory-mapped to stream status (0x00000000@0x04): running
Writing S2MM transfer length...
Stream to memory-mapped status (0x00000000@0x34): running
Writing MM2S transfer length...
Memory-mapped to stream status (0x00000000@0x04): running
Waiting for MM2S synchronization...
Waiting for S2MM sychronization...
Stream to memory-mapped status (0x00001002@0x34): running idle IOC_Irq
Memory-mapped to stream status (0x00001002@0x04): running idle IOC_Irq
Destination memory block: 44332211 00000000 00000000 00000000 00000000 00000000 00000000 00000000
```

# References
1. <a href="https://github.com/ikwzm/udmabuf/tree/master">u-dma-buf</a>

2. <a href="https://github.com/ikwzm/udmabuf/issues/74">u-dma-buf example designs</a>

3. <a href="https://github.com/ikwzm/FPGA-SoC-Linux-Example-2-ZYBO-Z7/tree/master?tab=readme-ov-file">FPGA SoC Linux Example 2 ZYBO Z7</a>

4. <a href="https://github.com/ikwzm/u-dma-buf-mgr">U-DMA-BUF-MGR</a>

5. <a href="https://github.com/ikwzm/PLBRAM-Ultra96/tree/master">PLBRAM Ultra96</a>
