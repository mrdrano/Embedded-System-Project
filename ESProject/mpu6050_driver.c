#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdint.h>

#define MPU6050_ADDR 0x68
#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B

int main() {
    int fd;
    char *bus = "/dev/i2c-1";
    uint8_t buf[14];

    // 1. 打開 I2C 設備檔案 (模擬存取 AXI Slave 埠)
    if ((fd = open(bus, O_RDWR)) < 0) {
        perror("Failed to open the bus");
        return 1;
    }

    // 2. 設定從端地址
    ioctl(fd, I2C_SLAVE, MPU6050_ADDR);

    // 3. WHO_AM_I 檢查 (驗證硬體通訊)
    uint8_t reg = REG_WHO_AM_I;
    write(fd, &reg, 1);
    uint8_t data;
    read(fd, &data, 1);

    printf("MPU6050 WHO_AM_I: 0x%02X\n", data);
    if (data != 0x68) {
        printf("Device error! Expected 0x68\n");
        return 1;
    }

    // 4. 解除睡眠模式 (喚醒晶片)
    uint8_t wake_up[] = {REG_PWR_MGMT_1, 0x00};
    write(fd, wake_up, 2);
    printf("Sensor awakened.\n");

    // 5. 循環讀取加速度數據 (模擬類 AXI-Stream 輸入)
    while (1) {
        uint8_t start_reg = REG_ACCEL_XOUT_H;
        write(fd, &start_reg, 1);
        
        // Burst Read: 一次讀取 6 Bytes (X, Y, Z 各 2 Bytes)
        if (read(fd, buf, 6) == 6) {
            int16_t x = (buf[0] << 8) | buf[1];
            int16_t y = (buf[2] << 8) | buf[3];
            int16_t z = (buf[4] << 8) | buf[5];
            printf("\rAccel X: %6d | Y: %6d | Z: %6d", x, y, z);
            fflush(stdout);
        }
        usleep(1000); // 模擬 1kHz 採樣
    }
    return 0;
}