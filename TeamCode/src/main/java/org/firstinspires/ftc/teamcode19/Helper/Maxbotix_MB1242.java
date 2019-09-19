package org.firstinspires.ftc.teamcode19.Helper;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

//Programmed following https://github.com/ftctechnh/ftc_app/wiki/Writing-an-I2C-Driver.
//Written by Tom Gao.
@I2cDeviceType()
@DeviceProperties(name = "Maxbotix MB1242",description = "Ultrasonic sensor from Maxbotix",xmlTag = "MB1242")
@Deprecated
public class Maxbotix_MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static byte address = 112;

    public Maxbotix_MB1242(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setLogging(true);
        this.deviceClient.setLoggingTag("mb1242_TOM");
        this.deviceClient.setI2cAddress(new I2cAddr(address));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected Maxbotix_MB1242(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    public void takeReading(byte addr){
        deviceClient.write(addr, new byte[]{(byte)81});
    }
    public int readRange(){
        byte upper = deviceClient.read(0x00,1)[0];
        byte lower = deviceClient.read(0x00,1)[0];
        return (upper * 256) + lower;
    }

    public void changeAddr(byte newAddress){
        deviceClient.write8(address, 170, I2cWaitControl.WRITTEN);
        deviceClient.write8(address, 165);
        deviceClient.write8(address, newAddress);
        address = newAddress;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "Maxbotix MB1242";
    }

    public String getLog(){
        return this.deviceClient.getConnectionInfo();
    }

}
