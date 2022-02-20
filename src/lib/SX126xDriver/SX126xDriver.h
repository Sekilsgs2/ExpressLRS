#pragma once
#include "targets.h"

class SX126xDriver
{
public:
    static SX126xDriver *instance;

    ///////Callback Function Pointers/////
    void (*RXdoneCallback)(); //function pointer for callback
    void (*TXdoneCallback)(); //function pointer for callback

    ///////////Radio Variables////////
    #define TXRXBuffSize 16
    volatile WORD_ALIGNED_ATTR uint8_t TXdataBuffer[TXRXBuffSize];
    volatile WORD_ALIGNED_ATTR uint8_t RXdataBuffer[TXRXBuffSize];

    uint32_t timeout = 0xFFFFFF;

    uint32_t currFreq;
    uint8_t PayloadLength;
    bool IQinverted;
	uint8_t currBW;
    ///////////////////////////////////

    /////////////Packet Stats//////////
    int8_t LastPacketRSSI;
    int8_t LastPacketSNR;

    ////////////////Configuration Functions/////////////
    SX126xDriver();
    bool Begin();
    void End();
    void SetTxIdleMode() { SetMode(SX126X_MODE_FS); }; // set Idle mode used when switching from RX to TX
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength, uint32_t interval,
                uint32_t flrcSyncWord=0, uint16_t flrcCrcSeed=0, uint8_t flrc=0);
    void SetFrequencyHz(uint32_t freq);
    void SetFrequencyReg(uint32_t freq);
    void SetRxTimeoutUs(uint32_t interval);
    void SetOutputPower(int8_t power);
    void SetOutputPowerMax() { SetOutputPower(15); };
	void SetPPMoffsetReg(int32_t offset);
	void SetSyncWord(uint8_t syncWord);

    int32_t GetFrequencyError();

    void TXnb();
    void RXnb();

    uint16_t GetIrqStatus();
    void ClearIrqStatus(uint16_t irqMask);

    void GetStatus();

    bool GetFrequencyErrorbool();
    uint8_t GetRxBufferAddr();
    int8_t GetRssiInst();
    void GetLastPacketStats();

private:
    SX126X_RadioOperatingModes_t currOpmode = SX126X_MODE_SLEEP;
	uint8_t currSyncWord = SX127X_SYNC_WORD;

    void SetMode(SX126X_RadioOperatingModes_t OPmode);
    void SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
	void SetDio3AsTcxoCtrl(float voltage, uint32_t delay = 5000);
	uint16_t getDeviceErrors();
	void clearDeviceErrors();
	void setWorkaround500Khz();
	void setWorkaroundIQPolarity();
	void setWorkaroundRXTimeout();

    // LoRa functions
    void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr);
    void SetPacketParamsLoRa(uint8_t PreambleLength, SX126X_RadioLoRaPacketLengthsModes_t HeaderType,
                             uint8_t PayloadLength, SX126X_RadioLoRaCrcModes_t crc,
                             uint8_t InvertIQ);

    void SetDioIrqParams(uint16_t irqMask,
                         uint16_t dio1Mask=SX126X_IRQ_RADIO_NONE,
                         uint16_t dio2Mask=SX126X_IRQ_RADIO_NONE,
                         uint16_t dio3Mask=SX126X_IRQ_RADIO_NONE);


    static void IsrCallback();
    void RXnbISR(); // ISR for non-blocking RX routine
    void TXnbISR(); // ISR for non-blocking TX routine
};
