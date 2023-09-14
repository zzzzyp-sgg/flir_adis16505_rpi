#include "adis16505/adis16505.h"

bool ADIS16505::setUp() {
    if (!bcm2835_init()){
      printf("bcm2835_init failed! Are you running as root?\n");
      return false;
    }

    if (!bcm2835_spi_begin()) {
        ROS_WARN("bcm2835_spi_begin failed! Are you running as root?\n");
        init_bcm_ = false;
    } else {
        ROS_INFO("SPI connection is successful.\n");
    }
    
    // 初始化设备
    bool isInit = initADIS16505();

    // 连接信息的输出提示
    if (isInit) {
        ROS_INFO("Connected to SPI device.\n");
    } else {
        ROS_WARN("Not connected to SPI device.\n");
        return false;
    }

    return true;
}

void ADIS16505::adisSetParam(int dec_rate, int filter) {
    dec_rate_ = dec_rate;
    filter_ = filter;
    ROS_INFO("DEC_RATE is set to:%d, FILTER is set to:%d\n", dec_rate_, filter_);
}

void ADIS16505::closeSPI() {
    bcm2835_spi_end();
    bcm2835_close();
}

bool ADIS16505::initADIS16505()
{
    // 高位优先
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    // 模式2,CPOL = 1 (polarity), CPHA = 1 (phase)
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
    // 时钟分频
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);

    // 选中设备，将这个引脚设置为输出模式
    bcm2835_gpio_fsel(CS_PIN, BCM2835_GPIO_FSEL_OUTP);
    // bcm2835_gpio_set(CS_PIN); 这里是设备未选中，先不用

    // 设备重启
    CS_LOW();
    bcm2835_delay(500);
    CS_HIGH();
    bcm2835_delay(500);

    // 判断是否连接成功
    // 启动后先读PROD_ID
    uint8_t rdat[2] = {0, 0};
    uint8_t wd[2] = {PROD_ID, 0x00};
    CS_LOW();
    bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd)); // 发送和接收数据
    CS_HIGH();
    bcm2835_delayMicroseconds(tSTALL);  // 延迟
    bool is_connected_ = isConnected();
    if (!is_connected_) return false;

    // 设置采样频率
    adisSetSampleRate();

    // 设置滤波等级
    /// TODO 关于这个等级的设置之后可以再看一下
    adisHardwareFilterSelect();

    return true;
}

bool ADIS16505::isConnected() {
    uint16_t prod_id = adisReadReg(PROD_ID);
    return (prod_id == 16505);
}

uint16_t ADIS16505::adisReadReg(uint16_t addr) {
    // 因为在SPI读取的时候，要延迟一帧才能接收到数据，所以在外面也要transfernb一次
    if (addr == BURST_CMD)
        return false;

    uint8_t wd[2] = {addr, 0x00};
    uint8_t rdat[2] = {0, 0};
    CS_LOW();
    bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
    CS_HIGH();
    bcm2835_delayMicroseconds(tSTALL);
    uint16_t val = (rdat[0] << 8) | rdat[1];
    return val;
}

bool ADIS16505::adisSetSampleRate() {
    // 设置DEC_RATE地址的值，也就是数据的更新频率
    adisWriteReg(DEC_RATE, dec_rate_);

    // 这里是又读了一遍，为了设置更新频率
    uint8_t wd[2] = {DEC_RATE, 0x00};
    uint8_t rdat[2] = {0, 0};
    CS_LOW();
    bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
    CS_HIGH();
    bcm2835_delayMicroseconds(tSTALL);
    auto tmp = adisReadReg(DEC_RATE);

    /* 频率 frequency = 2000 / (1 + DEC_RATE) (Hz) */
    frequency_ = 2000 / (1 + tmp);

    /// FIXME 加一个判断，判断频率是否设置正确了
    return true;
}

void ADIS16505::adisWriteReg(uint16_t addr, uint16_t val) {
    // 写地址的话是要写两个内容进去
    uint16_t txBuf1 = ((addr | 0x80) << 8) | (val & 0xFF);
    uint8_t tx_data1[2] = {txBuf1 >> 8, txBuf1 & 0xFF};
    uint16_t txBuf2 = ((addr + 1 | 0x80) << 8) | (0 & 0xFF);
    uint8_t tx_data2[2] = {txBuf2 >> 8, txBuf2 & 0xFF};

    uint8_t rdat[2] = {0, 0};
    CS_LOW();
    bcm2835_spi_transfernb(to_char(tx_data1), to_char(rdat), sizeof(tx_data1));
    CS_HIGH();
    bcm2835_delayMicroseconds(tSTALL);  
    CS_LOW();
    bcm2835_spi_transfernb(to_char(tx_data2), to_char(rdat), sizeof(tx_data2));
    CS_HIGH();
    bcm2835_delayMicroseconds(tSTALL);
}

void ADIS16505::adisHardwareFilterSelect() {
    // adisRegWrite16bit(FILT_CTRL, dat); // Set digital filter
    adisWriteReg(FILT_CTRL, filter_);
    uint8_t wd[2] = {FILT_CTRL, 0x00};
    uint8_t rdat[2] = {0, 0};
    CS_LOW();
    bcm2835_spi_transfernb(to_char(wd), to_char(rdat), sizeof(wd));
    CS_HIGH(); 
    bcm2835_delayMicroseconds(tSTALL);
    auto tmp = adisReadReg(FILT_CTRL);

    /// FIXME 加一个判断，判断滤波等级是否设置正确了
}

void ADIS16505::adisSingleRead() {
    // 先读取，都是两个16位合并成一个32位
    auto unused = adisReadReg(X_GYRO_LOW);  // 这里这里读到的是上一次发送的指令的内容
    auto x_g_l = adisReadReg(X_GYRO_OUT);   // X轴角速度
    auto x_g_o = adisReadReg(Y_GYRO_LOW);    
    auto y_g_l = adisReadReg(Y_GYRO_OUT);   // Y轴角速度
    auto y_g_o = adisReadReg(Z_GYRO_LOW);
    auto z_g_l = adisReadReg(Z_GYRO_OUT);   // Z轴角速度
    auto z_g_o = adisReadReg(X_ACCL_LOW);
    auto x_a_l = adisReadReg(X_ACCL_OUT);   // X轴加速度
    auto x_a_o = adisReadReg(Y_ACCL_LOW);
    auto y_a_l = adisReadReg(Y_ACCL_OUT);   // Y轴加速度
    auto y_a_o = adisReadReg(Z_ACCL_LOW);
    auto z_a_l = adisReadReg(Z_ACCL_OUT);   // Z轴加速度
    auto z_a_o = adisReadReg(TEMP_OUT);

    // 合并OUT和LOW
    gyro_raw_[0] = ((int32_t(x_g_o) << 16) + int32_t(x_g_l)) * M_PI / 180.0 / GYRO_SENSITIVITY;
    gyro_raw_[1] = ((int32_t(y_g_o) << 16) + int32_t(y_g_l)) * M_PI / 180.0 / GYRO_SENSITIVITY;
    gyro_raw_[2] = ((int32_t(z_g_o) << 16) + int32_t(z_g_l)) * M_PI / 180.0 / GYRO_SENSITIVITY;
    acc_raw_[0] = ((int32_t(x_a_o) << 16) + int32_t(x_a_l))  / ACCL_SENSITIVITY;
    acc_raw_[1] = ((int32_t(y_a_o) << 16) + int32_t(y_a_l))  / ACCL_SENSITIVITY;
    acc_raw_[2] = ((int32_t(z_a_o) << 16) + int32_t(z_a_l))  / ACCL_SENSITIVITY;
}