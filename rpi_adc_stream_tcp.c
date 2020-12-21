#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include "rpi_dma_utils.h"

#define VERSION "0.1"

#define SAMPLE_RATE     100     // Default & max sample rate (samples/sec)
#define MAX_SAMPLE_RATE 50000

// PWM definitions: divisor, and reload value
#define PWM_FREQ        1000000
#define PWM_VALUE       2

// ADC sample size (2 bytes, with 11 data bits)
#define ADC_RAW_LEN     2

// ADC and DAC chip-enables
#define ADC_CE_NUM      0
#define DAC_CE_NUM      1

// Definitions for 2 bytes per ADC sample (11-bit)
#define ADC_REQUEST(c)  {0xc0 | (c)<<5, 0x00}
#define ADC_VOLTAGE(n)  (((n) * 3.131) / 2048.0)
#define ADC_MILLIVOLTS(n) ((int)((((n) * 3300) + 1024) / 2048))
#define ADC_RAW_VAL(d)  (((uint16_t)(d)<<8 | (uint16_t)(d)>>8) & 0x7ff)

// Non-cached memory size
#define MAX_SAMPS       1024
#define SAMP_SIZE       4
#define BUFF_LEN        (MAX_SAMPS * SAMP_SIZE)
#define MAX_BUFFS       2
#define VC_MEM_SIZE     (PAGE_SIZE + (BUFF_LEN * MAX_BUFFS))

// DMA control block macros
#define NUM_CBS         10
#define REG(r, a)       REG_BUS_ADDR(r, a)
#define MEM(m, a)       MEM_BUS_ADDR(m, a)
#define CBS(n)          MEM_BUS_ADDR(mp, &dp->cbs[(n)])

// DMA transfer information for PWM and SPI
#define PWM_TI          (DMA_DEST_DREQ | (DMA_PWM_DREQ << 16) | DMA_WAIT_RESP)
#define SPI_RX_TI       (DMA_SRCE_DREQ | (DMA_SPI_RX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_DEST_INC)
#define SPI_TX_TI       (DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC)

// SPI clock frequency
#define MIN_SPI_FREQ    10000
#define MAX_SPI_FREQ    1000000
#define SPI_FREQ        1000000

// SPI 0 pin definitions
#define SPI0_CE0_PIN    8
#define SPI0_CE1_PIN    7
#define SPI0_MISO_PIN   9
#define SPI0_MOSI_PIN   10
#define SPI0_SCLK_PIN   11

// SPI registers and constants
#define SPI0_BASE       (PHYS_REG_BASE + 0x204000)
#define SPI_CS          0x00
#define SPI_FIFO        0x04
#define SPI_CLK         0x08
#define SPI_DLEN        0x0c
#define SPI_DC          0x14
#define SPI_FIFO_CLR    (3 << 4)
#define SPI_RX_FIFO_CLR (2 << 4)
#define SPI_TX_FIFO_CLR (1 << 4)
#define SPI_TFR_ACT     (1 << 7)
#define SPI_DMA_EN      (1 << 8)
#define SPI_AUTO_CS     (1 << 11)
#define SPI_RXD         (1 << 17)
#define SPI_CE0         0
#define SPI_CE1         1

// SPI register strings
char *spi_regstrs[] = {"CS", "FIFO", "CLK", "DLEN", "LTOH", "DC", ""};

// Microsecond timer
#define USEC_BASE       (PHYS_REG_BASE + 0x3000)
#define USEC_TIME       0x04
uint32_t usec_start;

// Buffer for streaming output, and raw Rx data
#define STREAM_BUFFLEN  10000
char stream_buff[STREAM_BUFFLEN];
uint32_t rx_buff[MAX_SAMPS];

// Virtual memory pointers to acceess peripherals & memory
extern MEM_MAP gpio_regs, dma_regs, clk_regs, pwm_regs;
MEM_MAP vc_mem, spi_regs, usec_regs;

// Data formats for -f option
#define FMT_USEC        1

void terminate(int sig);
void map_devices(void);

int init_spi(int hz);
void spi_cs(int set);
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len);
void spi_disable(void);

void adc_dma_init(MEM_MAP *mp, int n_samples, int single);
void adc_stream_start(void);
void adc_stream_wait(void);
void adc_stream_stop(void);
int adc_stream_csv(MEM_MAP *mp, char *value_string, int maxlen, int nsamples);
void do_streaming(MEM_MAP *mp, char *vals, int maxlen, int nsamp);

int in_chans=1, sample_count=0, sample_rate=SAMPLE_RATE;
uint32_t lockstep;

uint32_t pwm_range, overrun_total;

// Main program
int main(int argc, char *argv[])
{
    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);
    signal(SIGINT, terminate); // Terminate on Ctrl+C

    pwm_range = (PWM_FREQ * 2) / sample_rate;
    uint32_t spi_freq = init_spi(SPI_FREQ);

    lockstep = 0;
    sample_count = 64;

    printf("Streaming %u samples per block at %u S/s %s\n",
           sample_count, sample_rate, lockstep ? "(lockstep)" : "");
    adc_dma_init(&vc_mem, sample_count, 0);
    adc_stream_start();
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1)
        do_streaming(&vc_mem, stream_buff, STREAM_BUFFLEN, sample_count);
#pragma clang diagnostic pop
}

// Map GPIO, DMA and SPI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&spi_regs, (void *)SPI0_BASE, PAGE_SIZE);
    map_periph(&clk_regs, (void *)CLK_BASE, PAGE_SIZE);
    map_periph(&pwm_regs, (void *)PWM_BASE, PAGE_SIZE);
    map_periph(&usec_regs, (void *)USEC_BASE, PAGE_SIZE);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    terminate(0);
}

// Free memory & peripheral mapping and exit
void terminate(int sig)
{
    printf("Closing (Signal: %d)\n", sig);
    spi_disable();
    stop_dma(DMA_CHAN_A);
    stop_dma(DMA_CHAN_B);
    stop_dma(DMA_CHAN_C);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&usec_regs);
    unmap_periph_mem(&pwm_regs);
    unmap_periph_mem(&clk_regs);
    unmap_periph_mem(&spi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
//    if (fifo_name)
//        destroy_fifo(fifo_name, fifo_fd);
//    if (samp_total)
//        printf("Total samples %u, overruns %u\n", samp_total, overrun_total);
    if(overrun_total)
        printf("Overruns: %u\n", overrun_total);
    exit(0);
}

// Disable SPI
void spi_disable(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
    *REG32(spi_regs, SPI_CS) = 0;
}

// Initialise SPI0, given desired clock freq; return actual value
int init_spi(int hz)
{
    int f, div = (SPI_CLOCK_HZ / hz + 1) & ~1;

    gpio_set(SPI0_CE0_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_CE1_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_MISO_PIN, GPIO_ALT0, GPIO_PULLUP);
    gpio_set(SPI0_MOSI_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_SCLK_PIN, GPIO_ALT0, GPIO_NOPULL);
    while (div==0 || (f = SPI_CLOCK_HZ/div) > MAX_SPI_FREQ)
        div += 2;
    *REG32(spi_regs, SPI_CS) = 0x30;
    *REG32(spi_regs, SPI_CLK) = div;
    return(f);
}

// Set / clear SPI chip select
void spi_cs(int set)
{
    uint32_t csval = *REG32(spi_regs, SPI_CS);

    *REG32(spi_regs, SPI_CS) = set ? csval | 0x80 : csval & ~0x80;
}

// Transfer SPI bytes
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len)
{
    while (len--)
    {
        *REG8(spi_regs, SPI_FIFO) = *txd++;
        while((*REG32(spi_regs, SPI_CS) & (1<<17)) == 0) ;
        *rxd++ = *REG32(spi_regs, SPI_FIFO);
    }
}


typedef struct {
    DMA_CB cbs[NUM_CBS];
    uint32_t samp_size, pwm_val, adc_csd, txd[2];
    volatile uint32_t usecs[2], states[2], rxd1[MAX_SAMPS], rxd2[MAX_SAMPS];
} ADC_DMA_DATA;

// Initialise PWM-paced DMA for ADC sampling
void adc_dma_init(MEM_MAP *mp, int n_samples, int single)
{
    ADC_DMA_DATA *dp=mp->virt;
    ADC_DMA_DATA dma_data = {
            .samp_size = 2, .pwm_val = pwm_range, .txd={0xd0, in_chans>1 ? 0xf0 : 0xd0},
            .adc_csd = SPI_TFR_ACT | SPI_AUTO_CS | SPI_DMA_EN | SPI_FIFO_CLR | ADC_CE_NUM,
            .usecs = {0, 0}, .states = {0, 0}, .rxd1 = {0}, .rxd2 = {0},
            .cbs = {
                    // Rx input: read data from usec clock and SPI, into 2 ping-pong buffers
                    {SPI_RX_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[0]),  4,             0, CBS(1), 0}, // 0
                    {SPI_RX_TI, REG(spi_regs, SPI_FIFO),   MEM(mp, dp->rxd1),       n_samples * 4, 0, CBS(2), 0}, // 1
                    {SPI_RX_TI, REG(spi_regs, SPI_CS),     MEM(mp, &dp->states[0]), 4,             0, CBS(3), 0}, // 2
                    {SPI_RX_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[1]),  4,             0, CBS(4), 0}, // 3
                    {SPI_RX_TI, REG(spi_regs, SPI_FIFO),   MEM(mp, dp->rxd2),       n_samples * 4, 0, CBS(5), 0}, // 4
                    {SPI_RX_TI, REG(spi_regs, SPI_CS),     MEM(mp, &dp->states[1]), 4,             0, CBS(0), 0}, // 5
                    // Tx output: 2 data writes to SPI for chan 0 & 1, or both chan 0
                    {SPI_TX_TI, MEM(mp, dp->txd),          REG(spi_regs, SPI_FIFO), 8,             0, CBS(6), 0}, // 6
                    // PWM ADC trigger: wait for PWM, set sample length, trigger SPI
                    {PWM_TI,    MEM(mp, &dp->pwm_val),     REG(pwm_regs, PWM_FIF1), 4,             0, CBS(8), 0}, // 7
                    {PWM_TI,    MEM(mp, &dp->samp_size),   REG(spi_regs, SPI_DLEN), 4,             0, CBS(9), 0}, // 8
                    {PWM_TI,    MEM(mp, &dp->adc_csd),     REG(spi_regs, SPI_CS),   4,             0, CBS(7), 0}, // 9
            }
    };
    if (single)                                 // If single-shot, stop after first Rx block
        dma_data.cbs[2].next_cb = 0;
    memcpy(dp, &dma_data, sizeof(dma_data));    // Copy DMA data into uncached memory
    init_pwm(PWM_FREQ, pwm_range, PWM_VALUE);   // Initialise PWM, with DMA
    *REG32(pwm_regs, PWM_DMAC) = PWM_DMAC_ENAB | PWM_ENAB;
    *REG32(spi_regs, SPI_DC) = (8<<24) | (1<<16) | (8<<8) | 1;  // Set DMA priorities
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;                    // Clear SPI FIFOs
    start_dma(mp, DMA_CHAN_C, &dp->cbs[6], 0);  // Start SPI Tx DMA
    start_dma(mp, DMA_CHAN_B, &dp->cbs[0], 0);  // Start SPI Rx DMA
    start_dma(mp, DMA_CHAN_A, &dp->cbs[7], 0);  // Start PWM DMA, for SPI trigger
}

// Start ADC data acquisition
void adc_stream_start(void)
{
    start_pwm();
}


// Manage streaming output
void do_streaming(MEM_MAP *mp, char *vals, int maxlen, int nsamp)
{
    int n;

    if ((n=adc_stream_csv(mp, vals, maxlen, nsamp)) > 0)
    {
        printf("%s", vals);
//        if (!write_fifo(fifo_fd, vals, n))
//        {
//            printf("Stopped streaming\n");
//            close(fifo_fd);
//            fifo_fd = 0;
//            usleep(100000);
//        }
    }
    else
        usleep(10);
}

// Fetch samples from ADC buffer, return comma-delimited integer values
// If in lockstep mode, discard new data if FIFO isn't empty
int adc_stream_csv(MEM_MAP *mp, char *value_string, int maxlen, int nsamples)
{
    ADC_DMA_DATA *dp=mp->virt;
    uint32_t i, n, usec, string_len=0;
    for (n=0; n<2 && string_len == 0; n++)
    {
        if (dp->states[n])
        {
            memcpy(rx_buff, n ? (void *)dp->rxd2 : (void *)dp->rxd1, nsamples * 4);
            usec = dp->usecs[n];
            if (dp->states[n^1])
            {
                dp->states[0] = dp->states[1] = 0;
                overrun_total++;
                break;
            }
            dp->states[n] = 0;
            if (usec_start == 0)
                usec_start = usec;
            if (!lockstep)
            {
//              if (data_format == FMT_USEC) {
                    string_len += sprintf(&value_string[string_len], "%u", usec - usec_start); // print timestamp since start
//              }

                for (i=0; i < nsamples && string_len + 20 < maxlen; i++){
                    // print n samples (ADC-Values) comma-separated in one line
                    string_len += sprintf(&value_string[string_len], "%s%4.3f", string_len ? "," : "",
                                          ADC_VOLTAGE(ADC_RAW_VAL(rx_buff[i])));
                }
                string_len += sprintf(&value_string[string_len], "\n");

//                if (verbose)
//                { // print raw ADC-Values (bytes)
//                    for (int i=0; i<nsamples*4; i++)
//                        printf("%02X ", *(((uint8_t *)rx_buff)+i));
//                    printf("\n");
//                }
            }
        }
    }
    value_string[string_len] = 0;
    return(string_len);
}

// Stop ADC data acquisition
void adc_stream_stop(void)
{
    stop_pwm();
}
