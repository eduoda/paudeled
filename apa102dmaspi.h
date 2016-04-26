// DQ: http://dqsoft.blogspot.com.br/2016/02/arduino-due-dma-e-fita-de-led-c-apa102.html
// Rotinas para uso do SPI com DMA  
// Adaptadas de https://github.com/manitou48/DUEZoo/blob/master/dmaspi.ino  
  
/** chip select register number */  
#define SPI_CHIP_SEL 3  
/** DMAC transmit channel */  
#define SPI_DMAC_TX_CH  0  
/** DMAC Channel HW Interface Number for SPI TX. */  
#define SPI_TX_IDX  1  
  
/** Disable DMA Controller. */  
static void dmac_disable() {  
  DMAC->DMAC_EN &= (~DMAC_EN_ENABLE);  
}  
/** Enable DMA Controller. */  
static void dmac_enable() {  
  DMAC->DMAC_EN = DMAC_EN_ENABLE;  
}  
/** Disable DMA Channel. */  
static void dmac_channel_disable(uint32_t ul_num) {  
  DMAC->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;  
}  
/** Enable DMA Channel. */  
static void dmac_channel_enable(uint32_t ul_num) {  
  DMAC->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;  
}  
  
/** Poll for transfer complete. */  
static bool dmac_channel_transfer_done(uint32_t ul_num) {  
  return (DMAC->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) ? false : true;  
}  
  
// start TX DMA  
void spiDmaTX(const uint8_t* src, uint16_t count) {  
  static uint8_t ff = 0XFF;  
  uint32_t src_incr = DMAC_CTRLB_SRC_INCR_INCREMENTING;  
  if (!src) {  
    src = &ff;  
    src_incr = DMAC_CTRLB_SRC_INCR_FIXED;  
  }  
  dmac_channel_disable(SPI_DMAC_TX_CH);  
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_SADDR = (uint32_t)src;  
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DADDR = (uint32_t)&SPI0->SPI_TDR;  
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_DSCR =  0;  
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLA = count |  
    DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_BYTE;  
  
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CTRLB =  DMAC_CTRLB_SRC_DSCR |  
    DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC |  
    src_incr | DMAC_CTRLB_DST_INCR_FIXED;  
  
  DMAC->DMAC_CH_NUM[SPI_DMAC_TX_CH].DMAC_CFG = DMAC_CFG_DST_PER(SPI_TX_IDX) |  
      DMAC_CFG_DST_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ALAP_CFG;  
  
  dmac_channel_enable(SPI_DMAC_TX_CH);  
}  
  
// Prepara para o uso da interface SPI  
static void spiBegin() {  
  PIO_Configure(  
      g_APinDescription[PIN_SPI_MOSI].pPort,  
      g_APinDescription[PIN_SPI_MOSI].ulPinType,  
      g_APinDescription[PIN_SPI_MOSI].ulPin,  
      g_APinDescription[PIN_SPI_MOSI].ulPinConfiguration);  
  PIO_Configure(  
      g_APinDescription[PIN_SPI_MISO].pPort,  
      g_APinDescription[PIN_SPI_MISO].ulPinType,  
      g_APinDescription[PIN_SPI_MISO].ulPin,  
      g_APinDescription[PIN_SPI_MISO].ulPinConfiguration);  
  PIO_Configure(  
      g_APinDescription[PIN_SPI_SCK].pPort,  
      g_APinDescription[PIN_SPI_SCK].ulPinType,  
      g_APinDescription[PIN_SPI_SCK].ulPin,  
      g_APinDescription[PIN_SPI_SCK].ulPinConfiguration);  
  pmc_enable_periph_clk(ID_SPI0);  
  pmc_enable_periph_clk(ID_DMAC);  
  dmac_disable();  
  DMAC->DMAC_GCFG = DMAC_GCFG_ARB_CFG_FIXED;  
  dmac_enable();  
}  
  
//  configura a interface SPI  
//  velocidade = 84/spiRate MHz  
static void spiInit(uint8_t spiRate) {  
  Spi* pSpi = SPI0;  
  uint8_t scbr = 255;  
  if (spiRate > 0) {  
    scbr = spiRate;  
  }  
  //  disable SPI  
  pSpi->SPI_CR = SPI_CR_SPIDIS;  
  // reset SPI  
  pSpi->SPI_CR = SPI_CR_SWRST;  
  // no mode fault detection, set master mode  
  pSpi->SPI_MR = SPI_PCS(SPI_CHIP_SEL) | SPI_MR_MODFDIS | SPI_MR_MSTR;  
  // mode 0, 8-bit,  
  pSpi->SPI_CSR[SPI_CHIP_SEL] = SPI_CSR_SCBR(scbr) | SPI_CSR_NCPHA;  
  // enable SPI  
  pSpi->SPI_CR |= SPI_CR_SPIEN;  
}  
  
// Aguarda fim do envio anterior e dispara novo envio  
static void spiSend(const uint8_t* buf, size_t len) {  
  Spi* pSpi = SPI0;  
  while (!dmac_channel_transfer_done(SPI_DMAC_TX_CH)) {  
  }  
  while ((pSpi->SPI_SR & SPI_SR_TXEMPTY) == 0) {  
  }  
  // leave RDR empty  
  uint8_t b = pSpi->SPI_RDR;  
  spiDmaTX(buf, len);  
}

