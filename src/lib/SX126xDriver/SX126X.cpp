#include "SX126xDriver.h"
#include "logging.h"

SX126XHal hal;
SX126xDriver *SX126xDriver::instance = NULL;
static bool ImageCalibrated = false;

#define DEBUG_SX126X_OTA_TIMING

#if defined(DEBUG_SX126X_OTA_TIMING)
uint32_t beginTX;
uint32_t endTX;
#endif

const uint8_t SX127x_AllowedSyncwords[105] =
    {0, 5, 6, 7, 11, 12, 13, 15, 18,
     21, 23, 26, 29, 30, 31, 33, 34,
     37, 38, 39, 40, 42, 44, 50, 51,
     54, 55, 57, 58, 59, 61, 63, 65,
     67, 68, 71, 77, 78, 79, 80, 82,
     84, 86, 89, 92, 94, 96, 97, 99,
     101, 102, 105, 106, 109, 111, 113, 115,
     117, 118, 119, 121, 122, 124, 126, 127,
     129, 130, 138, 143, 161, 170, 172, 173,
     175, 180, 181, 182, 187, 190, 191, 192,
     193, 196, 199, 201, 204, 205, 208, 209,
     212, 213, 219, 220, 221, 223, 227, 229,
     235, 239, 240, 242, 243, 246, 247, 255};

/*
 * Period Base from table 11-24, page 79 datasheet rev 3.2
 * SX126X_RADIO_TICK_SIZE_0015_US = 15625 nanos
 * SX126X_RADIO_TICK_SIZE_0062_US = 62500 nanos
 * SX126X_RADIO_TICK_SIZE_1000_US = 1000000 nanos
 * SX126X_RADIO_TICK_SIZE_4000_US = 4000000 nanos
 */
#define RX_TIMEOUT_PERIOD_BASE SX126X_RADIO_TICK_SIZE_0015_US
#define RX_TIMEOUT_PERIOD_BASE_NANOS 15625

void ICACHE_RAM_ATTR nullCallback(void) {}

void SX126xCalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    hal.WriteCommand(SX126X_RADIO_CALIBRATEIMAGE, calFreq, 2 );
}

SX126xDriver::SX126xDriver()
{
    instance = this;
}

void SX126xDriver::End()
{
    SetMode(SX126X_MODE_SLEEP);
    hal.end();
    TXdoneCallback = &nullCallback; // remove callbacks
    RXdoneCallback = &nullCallback;
    currFreq = 868000000;
    PayloadLength = 8; // Dummy default value which is overwritten during setup.
}

bool SX126xDriver::Begin()
{
	hal.IsrCallback = &SX126xDriver::IsrCallback;

    hal.init();
    hal.reset();
	
	hal.WriteCommand(SX126X_RADIO_SET_REGULATORMODE, SX126X_USE_DCDC);     // Enable DCDC converter instead of LDO
	SetMode(SX126X_MODE_STDBY_RC);
	hal.WriteCommand( SX126X_RADIO_SET_RFSWITCHMODE, 1); //dio2 as rf switch
	
    delay(100);
	
    hal.WriteCommand(SX126X_RADIO_SET_PACKETTYPE, SX126X_PACKET_TYPE_LORA);                                                       //Set packet type to LoRa
    ConfigModParamsLoRa(SX126X_LORA_BW_500, SX126X_LORA_SF7, SX126X_LORA_CR_4_7);                                                
    SetPacketParamsLoRa(8, SX126X_LORA_PACKET_IMPLICIT, 8, SX126X_LORA_CRC_OFF, SX126X_LORA_IQ_NORMAL);                          //default params
    SetFIFOaddr(0x00, 0x00);
	hal.WriteCommand( SX126X_RADIO_SET_STOPRXTIMERONPREAMBLE, false);
	SetSyncWord(currSyncWord);
    SetDioIrqParams(SX126X_IRQ_RADIO_NONE);
    return true;
}

bool SyncWordOk(uint8_t syncWord)
{
  for (unsigned int i = 0; i < sizeof(SX127x_AllowedSyncwords); i++)
  {
    if (syncWord == SX127x_AllowedSyncwords[i])
    {
      return true;
    }
  }
  return false;
}

void SX126xDriver::SetSyncWord(uint8_t syncWord)
{
  uint8_t _syncWord = syncWord;

  while (SyncWordOk(_syncWord) == false)
  {
    _syncWord++;
  }

  if(syncWord != _syncWord){
    DBGLN("Using syncword: %d instead of: %d", _syncWord, syncWord);
  }

  hal.WriteRegister(SX126X_REG_LR_SYNCWORD, _syncWord);
  currSyncWord = _syncWord;
}

uint16_t SX126xDriver::getDeviceErrors() {
  uint8_t data[2] = {0, 0};
  hal.ReadCommand(SX126X_RADIO_GET_ERROR, data, 2);
  uint16_t opError = (((uint16_t)data[0] & 0xFF) << 8) & ((uint16_t)data[1]);
  return(opError);
}

void SX126xDriver::clearDeviceErrors() {
  uint8_t data[2] = {0, 0};
  hal.WriteCommand(SX126X_RADIO_CLR_ERROR, data, 2);
  return;
}


void SX126xDriver::SetDio3AsTcxoCtrl(float voltage, uint32_t delay)
{
  SetMode(SX126X_MODE_STDBY_RC);

  // check SX126X_XOSC_START_ERR flag and clear it
  if(getDeviceErrors() & SX126X_XOSC_START_ERR) {
	INFOLN("START SX126X_XOSC_START_ERR!");
    clearDeviceErrors();
  }

  // check 0 V disable
  if(fabs(voltage - 0.0) <= 0.001) {
    return(hal.reset());
  }

  // check alowed voltage values
  uint8_t data[4];
  if(fabs(voltage - 1.6) <= 0.001) {
    data[0] = SX126X_TCXO_CTRL_1_6V;
  } else if(fabs(voltage - 1.7) <= 0.001) {
    data[0] = SX126X_TCXO_CTRL_1_7V;
  } else if(fabs(voltage - 1.8) <= 0.001) {
    data[0] = SX126X_TCXO_CTRL_1_8V;
  } else if(fabs(voltage - 2.2) <= 0.001) {
    data[0] = SX126X_TCXO_CTRL_2_2V;
  } else if(fabs(voltage - 2.4) <= 0.001) {
    data[0] = SX126X_TCXO_CTRL_2_4V;
  } else if(fabs(voltage - 2.7) <= 0.001) {
    data[0] = SX126X_TCXO_CTRL_2_7V;
  } else if(fabs(voltage - 3.0) <= 0.001) {
    data[0] = SX126X_TCXO_CTRL_3_0V;
  } else if(fabs(voltage - 3.3) <= 0.001) {
    data[0] = SX126X_TCXO_CTRL_3_3V;
  } else {
	  INFOLN("SX126X WRONG TCXO VOLTAGE!");
    return;
  }

  // calculate delay
  uint32_t delayValue = (float)delay / 15.625;
  data[1] = (uint8_t)((delayValue >> 16) & 0xFF);
  data[2] = (uint8_t)((delayValue >> 8) & 0xFF);
  data[3] = (uint8_t)(delayValue & 0xFF);

  hal.WriteCommand( SX126X_RADIO_SET_TCXOMODE, data, 4 );
  
  if(getDeviceErrors() & SX126X_XOSC_START_ERR) {
	INFOLN("END SX126X_XOSC_START_ERR!");
    clearDeviceErrors();
  }
}

void SX126xDriver::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                          uint8_t PreambleLength, bool InvertIQ, uint8_t _PayloadLength, uint32_t interval,
                          uint32_t flrcSyncWord, uint16_t flrcCrcSeed, uint8_t flrc)
{
    uint8_t irqs = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE;
    uint8_t const mode = (flrc) ? SX126X_PACKET_TYPE_GFSK : SX126X_PACKET_TYPE_LORA;

    PayloadLength = _PayloadLength;
    IQinverted = InvertIQ;
	currBW = bw;
    SetMode(SX126X_MODE_STDBY_RC);
    hal.WriteCommand(SX126X_RADIO_SET_PACKETTYPE, mode);
    if (mode == SX126X_PACKET_TYPE_GFSK)
    {
        DBGLN("Config FLRC");

    }
    else
    {
        DBGLN("Config LoRa");
        ConfigModParamsLoRa(bw, sf, cr);
        SetPacketParamsLoRa(PreambleLength, SX126X_LORA_PACKET_IMPLICIT,
                            _PayloadLength, SX126X_LORA_CRC_OFF, InvertIQ);
    }
	setWorkaroundIQPolarity();
    SetFrequencyReg(freq);
    SetDioIrqParams(irqs, irqs);
    SetRxTimeoutUs(interval);
	hal.WriteCommand( SX126X_RADIO_SET_LORASYMBTIMEOUT, 0);
}

void SX126xDriver::SetRxTimeoutUs(uint32_t interval)
{
    if (interval)
    {
        timeout = interval * 1000 / RX_TIMEOUT_PERIOD_BASE_NANOS; // number of periods for the SX126X to timeout
    }
    else
    {
        timeout = 0xFFFFFF;   // no timeout, continuous mode
    }
}

void SX126xDriver::setWorkaround500Khz()
{
	if (currBW == SX126X_LORA_BW_500)
		hal.WriteRegister( SX126X_REG_TX_MODULATION, hal.ReadRegister( SX126X_REG_TX_MODULATION ) & (0xFB) );
}

void SX126xDriver::setWorkaroundRXTimeout()
{
	hal.WriteRegister( SX126X_REG_RTC_CTRL, 0x00 );
	hal.WriteRegister( SX126X_REG_EVT_CLR, hal.ReadRegister( SX126X_REG_EVT_CLR ) & (0x02) );
}

void SX126xDriver::setWorkaroundIQPolarity()
{
	if (IQinverted)
		hal.WriteRegister( SX126X_REG_IQ_CONFIG, hal.ReadRegister( SX126X_REG_IQ_CONFIG ) | (0x04) );
	else
		hal.WriteRegister( SX126X_REG_IQ_CONFIG, hal.ReadRegister( SX126X_REG_IQ_CONFIG ) & (0xFB) );
}

void SX126xDriver::SetOutputPower(int8_t power)
{
    // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
    // RegTxClampConfig = @address 0x08D8	
	hal.WriteRegister( SX126X_REG_TX_CLAMP_CONFIG, hal.ReadRegister( SX126X_REG_TX_CLAMP_CONFIG ) | ( 0x1E ) );
	// WORKAROUND END
	uint8_t buf[4];

    buf[0] = 0x04;
    buf[1] = 0x07;
    buf[2] = 0x00;
    buf[3] = 0x01;
    hal.WriteCommand( SX126X_RADIO_SET_PACONFIG, buf, 4);
	
	hal.WriteRegister( SX126X_REG_OCP, 0x38); //SET OCP max to 140ma
	
	buf[0] = map(power, 0,15,0,22);
    buf[1] = ( uint8_t )SX126X_RADIO_RAMP_10_US;
    hal.WriteCommand(SX126X_RADIO_SET_TXPARAMS, buf, 2);
    DBGLN("SetPower: %d", buf[0]);
    return;
}

void SX126xDriver::SetMode(SX126X_RadioOperatingModes_t OPmode)
{
    if (OPmode == currOpmode)
    {
       return;
    }

    uint8_t buf[3];

    switch (OPmode)
    {

    case SX126X_MODE_SLEEP:
        hal.WriteCommand(SX126X_RADIO_SET_SLEEP, 0x01);
        break;

    case SX126X_MODE_STDBY_RC:
        hal.WriteCommand(SX126X_RADIO_SET_STANDBY, SX126X_STDBY_RC);
        break;

    case SX126X_MODE_STDBY_XOSC:
        hal.WriteCommand(SX126X_RADIO_SET_STANDBY, SX126X_STDBY_XOSC);
        break;

    case SX126X_MODE_FS:
        hal.WriteCommand(SX126X_RADIO_SET_FS, 0x00);
        break;

    case SX126X_MODE_RX:
        buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
        buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
        buf[2] = ( uint8_t )( timeout & 0xFF );
		hal.WriteRegister(SX126X_REG_RX_GAIN, 0x96 ); 
        hal.WriteCommand(SX126X_RADIO_SET_RX, buf, 3);
        break;

    case SX126X_MODE_TX:
		setWorkaround500Khz();
        buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
        buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
        buf[2] = ( uint8_t )( timeout & 0xFF );
        hal.WriteCommand(SX126X_RADIO_SET_TX, buf, 3);
        break;

    case SX126X_MODE_CAD:
        break;

    default:
        break;
    }
	
    currOpmode = OPmode;
}

void SX126xDriver::ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t buf[4] = { 0x00, 0x00, 0x00, 0x00};
    buf[0] = sf;
    buf[1] = bw;
    buf[2] = cr;
    buf[3] = 0;

    hal.WriteCommand(SX126X_RADIO_SET_MODULATIONPARAMS, buf, 4);

}

void SX126xDriver::SetPacketParamsLoRa(uint8_t PreambleLength, SX126X_RadioLoRaPacketLengthsModes_t HeaderType,
                                       uint8_t PayloadLength, SX126X_RadioLoRaCrcModes_t crc,
                                       uint8_t InvertIQ)
{
    uint8_t buf[6];

    buf[0] = ( PreambleLength >> 8 ) & 0xFF;
    buf[1] = PreambleLength;
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = crc;
    buf[5] = InvertIQ ? SX126X_LORA_IQ_NORMAL: SX126X_LORA_IQ_INVERTED;
	
    hal.WriteCommand(SX126X_RADIO_SET_PACKETPARAMS, buf, 6);
}

void ICACHE_RAM_ATTR SX126xDriver::SetFrequencyHz(uint32_t Reqfreq)
{
    uint8_t buf[4] = {0};
	
	SetMode(SX126X_MODE_STDBY_RC);

    if( ImageCalibrated == false )
    {
        SX126xCalibrateImage( Reqfreq );
        ImageCalibrated = true;
    }

    uint32_t freq = (uint32_t)((double)Reqfreq / (double)FREQ_STEP);
    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );
	
    hal.WriteCommand(SX126X_RADIO_SET_RFFREQUENCY, buf, 4);
    currFreq = Reqfreq;
}

void ICACHE_RAM_ATTR SX126xDriver::SetFrequencyReg(uint32_t freq)
{
    uint8_t buf[4] = {0};
	
	SetMode(SX126X_MODE_STDBY_RC);
	
    if( ImageCalibrated == false )
    {
        SX126xCalibrateImage((uint32_t)((double)freq * (double)FREQ_STEP));
        ImageCalibrated = true;
    }

    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );

    hal.WriteCommand(SX126X_RADIO_SET_RFFREQUENCY, buf, 4);
    currFreq = freq;
}

int32_t ICACHE_RAM_ATTR SX126xDriver::GetFrequencyError()
{
    return 0;
}

void SX126xDriver::SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    hal.WriteCommand(SX126X_RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf));
}

void SX126xDriver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    hal.WriteCommand(SX126X_RADIO_SET_DIOIRQPARAMS, buf, sizeof(buf));
}

uint16_t ICACHE_RAM_ATTR SX126xDriver::GetIrqStatus()
{
    uint8_t status[2];

    hal.ReadCommand(SX126X_RADIO_GET_IRQSTATUS, status, 2);
    return status[0] << 8 | status[1];
}

void ICACHE_RAM_ATTR SX126xDriver::ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    hal.WriteCommand(SX126X_RADIO_CLR_IRQSTATUS, buf, sizeof(buf));
}

void ICACHE_RAM_ATTR SX126xDriver::TXnbISR()
{	
	uart_cmd(UART0, ENABLE);
    currOpmode = SX126X_MODE_FS; // radio goes to FS after TX
#ifdef DEBUG_SX126X_OTA_TIMING
    endTX = micros()-beginTX;
#endif
    TXdoneCallback();
}

uint8_t FIFOaddr = 0;

void ICACHE_RAM_ATTR SX126xDriver::TXnb()
{	uart_cmd(UART0, DISABLE);
#ifdef DEBUG_SX126X_OTA_TIMING
    beginTX = micros();
#endif

	SetMode(SX126X_MODE_STDBY_RC);
	SetFIFOaddr(0x00, 0x00);	// do first to allow PA stablise
    hal.WriteBuffer(0x00, TXdataBuffer, PayloadLength); //todo fix offset to equal fifo addr
    SetMode(SX126X_MODE_TX);
}

extern volatile uint32_t rxisr_t;

void ICACHE_RAM_ATTR SX126xDriver::RXnbISR()
{
    // In continuous receive mode, the device stays in Rx mode
    if (timeout != 0xFFFFFF)
    {
        currOpmode = SX126X_MODE_STDBY_RC;
		setWorkaroundRXTimeout();
    }
    uint8_t FIFOaddr = GetRxBufferAddr();
    hal.ReadBuffer(FIFOaddr, RXdataBuffer, PayloadLength);
    GetLastPacketStats();
    RXdoneCallback();
}

void ICACHE_RAM_ATTR SX126xDriver::RXnb()
{
	SetMode(SX126X_MODE_STDBY_RC);
	SetFIFOaddr(0x00, 0x00);
    SetMode(SX126X_MODE_RX);
}

uint8_t ICACHE_RAM_ATTR SX126xDriver::GetRxBufferAddr()
{
    WORD_ALIGNED_ATTR uint8_t status[2] = {0};
    hal.ReadCommand(SX126X_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

void ICACHE_RAM_ATTR SX126xDriver::GetStatus()
{
    uint8_t status = 0;
    hal.ReadCommand(SX126X_RADIO_GET_STATUS, (uint8_t *)&status, 1);
    DBGLN("Status: %x, %x, %x", (0b11100000 & status) >> 5, (0b00011100 & status) >> 2, 0b00000001 & status);
}

bool ICACHE_RAM_ATTR SX126xDriver::GetFrequencyErrorbool()
{
    return 0;
}

int8_t ICACHE_RAM_ATTR SX126xDriver::GetRssiInst()
{
    uint8_t status = 0;

    hal.ReadCommand(SX126X_RADIO_GET_RSSIINST, (uint8_t *)&status, 1);
    return -(int8_t)(status >> 1);
}

void ICACHE_RAM_ATTR SX126xDriver::GetLastPacketStats()
{
    uint8_t status[3];

    hal.ReadCommand(SX126X_RADIO_GET_PACKETSTATUS, status, sizeof(status));
    LastPacketRSSI = -status[2] >> 1;;
    LastPacketSNR = ( status[1] < 128 ) ? ( status[1] >> 2 ) : (( ( status[1] - 256 ) >> 2 ) );;
}

void ICACHE_RAM_ATTR SX126xDriver::SetPPMoffsetReg(int32_t offset)
{
	return;
}

void ICACHE_RAM_ATTR SX126xDriver::IsrCallback()
{
    uint16_t irqStatus = instance->GetIrqStatus();
	
    instance->ClearIrqStatus(SX126X_IRQ_RADIO_ALL);
    if (irqStatus & SX126X_IRQ_TX_DONE)
    {
        hal.TXRXdisable();
        instance->TXnbISR();
    }
    if (irqStatus & SX126X_IRQ_RX_DONE) {
        instance->RXnbISR();
	}
}
