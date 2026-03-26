/*
 * dma_info.c  —  CLI command: show-dma
 *
 * Displays live register state for every DMA1/DMA2 stream that is
 * configured in this project, plus notes on peripherals that use their
 * own internal DMA engines.
 *
 * Active streams
 * ─────────────────────────────────────────────────────────────────
 *  DMA1  Stream 5  Ch 7   M→P  Circ   DAC1 DHR12R1  (sine, PA4)
 *  DMA2  Stream 2  Ch 1   P→M  Circ   ADC2 DR       (3-ch scan)
 *
 * Not on DMA1/2
 * ─────────────────────────────────────────────────────────────────
 *  ETH   — MAC contains its own internal DMA controller
 *  USB   — OTG-FS hardware DMA is explicitly disabled in this project
 *
 * References: RM0090 Rev 19, §10 (DMA), Tables 42-43 (stream requests)
 */

#include "stm32f4xx.h"
#include "printers.h"

/* ── column positions (characters from left margin) ──────────────── */
#define C_CH     9
#define C_DIR   12
#define C_MODE  18
#define C_PRI   23
#define C_ST    29
#define C_NDTR  33
#define C_PERI  39

/* ── local helpers ───────────────────────────────────────────────── */

static const char *dma_dir_str(uint32_t cr)
{
    switch ((cr & DMA_SxCR_DIR) >> DMA_SxCR_DIR_Pos) {
        case 0:  return "P->M";
        case 1:  return "M->P";
        case 2:  return "M->M";
        default: return "???";
    }
}

static const char *dma_pl_str(uint32_t cr)
{
    switch ((cr & DMA_SxCR_PL) >> DMA_SxCR_PL_Pos) {
        case 0:  return "Low";
        case 1:  return "Med";
        case 2:  return "High";
        case 3:  return "VHi";
        default: return "?";
    }
}

/*
 * Print one stream row.
 *   label      — e.g. "DMA1-S5"
 *   s          — pointer to the DMA_Stream_TypeDef
 *   periph     — short description of the connected peripheral
 *   isr        — value of the controller's LISR or HISR register
 *   feif_mask  — FEIF bit mask for this stream in that ISR register
 *   teif_mask  — TEIF bit mask for this stream in that ISR register
 */
static void print_stream_row(const char           *label,
                             DMA_Stream_TypeDef   *s,
                             const char           *periph,
                             uint32_t              isr,
                             uint32_t              feif_mask,
                             uint32_t              teif_mask)
{
    uint32_t cr    = s->CR;
    uint32_t ndtr  = s->NDTR;
    uint32_t ch    = (cr & DMA_SxCR_CHSEL) >> DMA_SxCR_CHSEL_Pos;
    int      en    = (cr & DMA_SxCR_EN)   != 0;
    int      circ  = (cr & DMA_SxCR_CIRC) != 0;
    int      fe    = (isr & feif_mask)     != 0;
    int      te    = (isr & teif_mask)     != 0;

    maybeCr();
    print(label);         tabTo(C_CH);
    printDec(ch);         tabTo(C_DIR);
    print(dma_dir_str(cr)); tabTo(C_MODE);
    print(circ ? "Circ" : "Once"); tabTo(C_PRI);
    print(dma_pl_str(cr)); tabTo(C_ST);
    print(en ? "run" : "off"); tabTo(C_NDTR);
    printDec(ndtr);       tabTo(C_PERI);
    print(periph);

    if (fe || te) {
        print("  [ERR:");
        if (fe) print(" FEIF");
        if (te) print(" TEIF");
        print("]");
    }
    printCr();
}

/* ── public CLI command ──────────────────────────────────────────── */

void show_dma(void)
{
    maybeCr();
    print("--- DMA usage (STM32F407) ---"); printCr();

    /* Clock-enable status (RCC AHB1) */
    int dma1_clk = (RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN) != 0;
    int dma2_clk = (RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN) != 0;
    maybeCr();
    print("DMA1 clock: "); print(dma1_clk ? "enabled" : "disabled");
    print("    DMA2 clock: "); print(dma2_clk ? "enabled" : "disabled");
    printCr();

    /* Column header */
    maybeCr();
    print("Stream"); tabTo(C_CH);
    print("Ch");     tabTo(C_DIR);
    print("Dir");    tabTo(C_MODE);
    print("Mode");   tabTo(C_PRI);
    print("Pri");    tabTo(C_ST);
    print("St");     tabTo(C_NDTR);
    print("NDTR");   tabTo(C_PERI);
    print("Peripheral"); printCr();

    print("-------  --  -----  ----  ---  ---  ----  -------------------------");
    printCr();

    /*
     * DMA1 Stream 5 Channel 7 → DAC1 DHR12R1
     * M→P  Circular  (see dac_sine.c)
     * HISR bit offsets for stream 5: FEIF5=6, TEIF5=9
     */
    print_stream_row("DMA1-S5",
                     DMA1_Stream5,
                     "DAC1 DHR12R1 (sine, PA4)",
                     DMA1->HISR,
                     DMA_HISR_FEIF5,
                     DMA_HISR_TEIF5);

    /*
     * DMA2 Stream 2 Channel 1 → ADC2 DR
     * P→M  Circular  (see adc_driver.c)
     * LISR bit offsets for stream 2: FEIF2=16, TEIF2=19
     */
    print_stream_row("DMA2-S2",
                     DMA2_Stream2,
                     "ADC2 DR (3-ch scan)",
                     DMA2->LISR,
                     DMA_LISR_FEIF2,
                     DMA_LISR_TEIF2);

    /* Peripherals that are NOT on the DMA1/2 fabric */
    maybeCr(); printCr();
    print("ETH: uses internal MAC DMA (not on DMA1/2 fabric)"); printCr();
    print("USB: OTG-FS hardware DMA disabled in this project"); printCr();
}
