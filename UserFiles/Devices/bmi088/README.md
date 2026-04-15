# STM32H7 DMA Buffer 链接脚本修改

STM32H7 的 DMA1/DMA2 无法访问 DTCMRAM (0x20000000)，SPI DMA 传输的 buffer 必须放在 RAM_D1 (0x24000000)。
CubeMX 重新生成代码后，需要在 `STM32H723XG_FLASH.ld` 中补回以下内容。

## 修改方法

在 `.bss` section 和 `._user_heap_stack` section 之间，添加：

```ld
  /* DMA-accessible buffer section in AXI SRAM (DMA1/DMA2 cannot access DTCM) */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(4);
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(4);
  } >RAM_D1
```
