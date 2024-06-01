#include "mcp2515.h"
#include "main.h"


#define MCP_16MHz_125kBPS_CFG1 (0x03)
#define MCP_16MHz_125kBPS_CFG2 (0xF0)
#define MCP_16MHz_125kBPS_CFG3 (0x86)

const RXBn_REGS RXB[N_RXBUFFERS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};


void MCP2515_init(MCP2515 *mcp, const uint8_t _CS, const uint32_t _SPI_CLOCK)
{

    mcp->SPICS = _CS;
    mcp->SPI_CLOCK = _SPI_CLOCK;

    // Initialize GPIO pins and any other hardware-specific setup
    // pinMode(mcp->SPICS, OUTPUT);
    // digitalWrite(mcp->SPICS, HIGH);
}

CAN_ERROR MCP2515_reset(SPI_HandleTypeDef* hspi1)
{
    startSPI();
    uint8_t instruction = INSTRUCTION_RESET;
    HAL_SPI_Transmit(hspi1, &instruction, 1, HAL_MAX_DELAY);
    endSPI();

    delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    setRegisters(MCP_TXB0CTRL, zeros, 14, hspi1);
    setRegisters(MCP_TXB1CTRL, zeros, 14, hspi1);
    setRegisters(MCP_TXB2CTRL, zeros, 14, hspi1);

    setRegister(MCP_RXB0CTRL, 0, hspi1);
    setRegister(MCP_RXB1CTRL, 0, hspi1);

    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF, hspi1);

    modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT, hspi1);
    modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT, hspi1);

    RXF filters[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
    for (int i = 0; i < 6; i++) {
        bool ext = (i == 1);
        CAN_ERROR result = setFilter(filters[i], ext, 0, hspi1);
        if (result != ERROR_OK) {
            return result;
        }
    }

    MASK masks[] = {0x00, 0x01};
    for (int i = 0; i < 2; i++) {
        CAN_ERROR result = setFilterMask(masks[i], true, 0, hspi1);
        if (result != ERROR_OK) {
            return result;
        }
    }

    return ERROR_OK;
}


CAN_ERROR set_bitrate_125kbps(SPI_HandleTypeDef* hspi1)
{

    CAN_ERROR error = setConfigMode(hspi1);
    if (error != ERROR_OK) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;

    cfg1 = MCP_16MHz_125kBPS_CFG1;
    cfg2 = MCP_16MHz_125kBPS_CFG2;
    cfg3 = MCP_16MHz_125kBPS_CFG3;

    if (set) {
            setRegister(MCP_CNF1, cfg1, hspi1);
            setRegister(MCP_CNF2, cfg2, hspi1);
            setRegister(MCP_CNF3, cfg3, hspi1);
            return ERROR_OK;
        }
    else {
        return ERROR_FAIL;
    }
}

CAN_ERROR setMode(const CANCTRL_REQOP_MODE mode, SPI_HandleTypeDef* hspi1)
{
    modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode, hspi1);

    unsigned long endTime = millis() + 10;
    bool modeMatch = false;
    while (millis() < endTime) {
        uint8_t newmode = readRegister(MCP_CANSTAT, hspi1);
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == mode;

        if (modeMatch) {
            break;
        }
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;

}

CAN_ERROR readMessageInternal(const RXBn rxbn, struct can_frame *frame, SPI_HandleTypeDef* hspi1)
{

	uint8_t tbufdata[5];

	readRegisters(RXB->SIDH, tbufdata, 5, hspi1);

	uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

	if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
	    id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
	    id = (id<<8) + tbufdata[MCP_EID8];
	    id = (id<<8) + tbufdata[MCP_EID0];
	    id |= CAN_EFF_FLAG;
	}

	uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
	if (dlc > CAN_MAX_DLEN) {
	    return ERROR_FAIL;
	}

	uint8_t ctrl = readRegister(RXB->CTRL, hspi1);
	if (ctrl & RXBnCTRL_RTR) {
	    id |= CAN_RTR_FLAG;
	}

	frame->can_id = id;
	frame->can_dlc = dlc;

	readRegisters(RXB->DATA, frame->data, dlc, hspi1);

	modifyRegister(MCP_CANINTF, RXB->CANINTF_RXnIF, 0, hspi1);

    return ERROR_OK;
}

CAN_ERROR readMessage(struct can_frame *frame, SPI_HandleTypeDef* hspi1)
{
    CAN_ERROR rc;
    uint8_t stat = getStatus(hspi1);

    if ( stat & STAT_RX0IF ) {
        rc = readMessageInternal(RXB0, frame, hspi1);
    } else if ( stat & STAT_RX1IF ) {
        rc = readMessageInternal(RXB1, frame, hspi1);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;
}


CAN_ERROR setConfigMode(SPI_HandleTypeDef* hspi1)
{
    return setMode(CANCTRL_REQOP_CONFIG, hspi1);
}

CAN_ERROR setListenOnlyMode(SPI_HandleTypeDef* hspi1)
{
    return setMode(CANCTRL_REQOP_LISTENONLY, hspi1);
}

CAN_ERROR setSleepMode(SPI_HandleTypeDef* hspi1)
{
    return setMode(CANCTRL_REQOP_SLEEP, hspi1);
}

CAN_ERROR setLoopbackMode(SPI_HandleTypeDef* hspi1)
{
    return setMode(CANCTRL_REQOP_LOOPBACK, hspi1);
}

CAN_ERROR setNormalMode(SPI_HandleTypeDef* hspi1)
{
    return setMode(CANCTRL_REQOP_NORMAL, hspi1);
}

uint8_t getStatus(SPI_HandleTypeDef* hspi1)
{

	uint8_t data;
	uint8_t instruction = INSTRUCTION_READ_STATUS;

    startSPI();

    HAL_SPI_Transmit(hspi1, &instruction, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
    endSPI();

    return data;
}

CAN_ERROR setFilter(const RXF num, const bool ext, const uint32_t ulData, SPI_HandleTypeDef* hspi1)
{
    CAN_ERROR res = setConfigMode(hspi1);
    if (res != ERROR_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);
    setRegisters(reg, tbufdata, 4, hspi1);

    return ERROR_OK;
}

CAN_ERROR setFilterMask(const MASK mask, const bool ext, const uint32_t ulData, SPI_HandleTypeDef* hspi1)
{
    CAN_ERROR res = setConfigMode(hspi1);
    if (res != ERROR_OK) {
        return res;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    setRegisters(reg, tbufdata, 4, hspi1);

    return ERROR_OK;
}

void prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
    } else {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}
void startSPI(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //start SPI communication
}

void endSPI(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //end SPI communication
}
uint8_t readRegister(const REGISTER reg, SPI_HandleTypeDef* hspi1)
{
	uint8_t ret;
	startSPI();
	uint8_t instruction = INSTRUCTION_READ;
	HAL_SPI_Transmit(hspi1, &instruction, 1, HAL_MAX_DELAY); //send instruction
	HAL_SPI_Transmit(hspi1, (uint8_t*)&reg, 1, HAL_MAX_DELAY); //send register
	HAL_SPI_Receive(hspi1, &ret, 1, HAL_MAX_DELAY); //receive data
	endSPI();
	return ret;
}
void readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n, SPI_HandleTypeDef* hspi1)
{
	startSPI();
	uint8_t instruction = INSTRUCTION_READ;
	HAL_SPI_Transmit(hspi1, &instruction, 1, HAL_MAX_DELAY); //send instruction
	HAL_SPI_Transmit(hspi1, (uint8_t*)&reg, 1, HAL_MAX_DELAY); //send register
	for (uint8_t i=0; i<n; i++){
		HAL_SPI_Receive(hspi1, &values[i], 1, HAL_MAX_DELAY); //receive data
	}
	endSPI();
}
void setRegister(const REGISTER reg, const uint8_t value, SPI_HandleTypeDef* hspi1)
{
	startSPI();
	uint8_t instruction = INSTRUCTION_WRITE;
	HAL_SPI_Transmit(hspi1, &instruction, 1, HAL_MAX_DELAY); //send instruction
	HAL_SPI_Transmit(hspi1, (uint8_t*)&reg, 1, HAL_MAX_DELAY); //send register
	HAL_SPI_Transmit(hspi1, (uint8_t*)&value, 1, HAL_MAX_DELAY); //send value
	endSPI();
}
void setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n, SPI_HandleTypeDef* hspi1)
{
	startSPI();
	uint8_t instruction = INSTRUCTION_WRITE;
	HAL_SPI_Transmit(hspi1, &instruction, 1, HAL_MAX_DELAY); //send instruction
	HAL_SPI_Transmit(hspi1, (uint8_t*)&reg, 1, HAL_MAX_DELAY); //send register
	for (uint8_t i=0; i<n; i++){
		HAL_SPI_Transmit(hspi1, (uint8_t*)&values[i], 1, HAL_MAX_DELAY); //receive data
	}
	endSPI();
}
void modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data, SPI_HandleTypeDef* hspi1)
{
	startSPI();
	uint8_t instruction = INSTRUCTION_BITMOD;
	HAL_SPI_Transmit(hspi1, &instruction, 1, HAL_MAX_DELAY); //send instruction
	HAL_SPI_Transmit(hspi1, (uint8_t*)&reg, 1, HAL_MAX_DELAY); //send register
	HAL_SPI_Transmit(hspi1, (uint8_t*)&mask, 1, HAL_MAX_DELAY); //send value
	HAL_SPI_Transmit(hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY); //send value
	endSPI();
}
