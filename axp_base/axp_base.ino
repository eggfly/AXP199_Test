/*
  MIT License

  Copyright (c) 2019 lewis he

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Stoftware without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include <Wire.h>
#include <axp20x.h>

AXP20X_Class axp;
bool  axpIrq = 0;

const uint8_t i2c_sda = 7;
const uint8_t i2c_scl = 6;
const uint8_t axp_irq_pin = 10;

void setFlag(void)
{
  axpIrq = true;
}


void printAXPStatus() {
  Serial.printf("ADC sampling rate=%d\n", axp.getAdcSamplingRate());
  Serial.printf("ACIN Current=%.3fmA, V=%.3fV\n", axp.getAcinCurrent(), axp.getAcinVoltage() / 1000);
  Serial.printf("VBUS Current=%.3fmA, V=%.3fV\n", axp.getVbusCurrent(), axp.getVbusVoltage() / 1000);

  Serial.print("DC1:");
  Serial.print(axp.isDCDC1Enable() ? String(axp.getDCDC1Voltage()) + " mv" : "DISABLE");
  Serial.print("  ");

  Serial.print("DC2:");
  Serial.print(axp.isDCDC2Enable() ? String(axp.getDCDC2Voltage()) + " mv" : "DISABLE");
  Serial.print("  ");

  Serial.print("DC3:");
  Serial.print(axp.isDCDC3Enable() ? String(axp.getDCDC3Voltage()) + " mv" : "DISABLE");
  Serial.print("  ");

  Serial.print("LDO2:");
  Serial.print(axp.isLDO2Enable() ? String(axp.getLDO2Voltage()) + " mv" : "DISABLE");
  Serial.print("  ");

  Serial.print("LDO3:");
  Serial.print(axp.isLDO3Enable() ? String(axp.getLDO3Voltage()) + " mv" : "DISABLE");
  Serial.print("  ");

  Serial.print("Exten:");
  Serial.print(axp.isExtenEnable() ? "ENABLE" : "DISABLE");
  Serial.print("\r\n");

  Serial.print("BAT Volate:");
  Serial.print(axp.getBattVoltage());
  Serial.println(" mV");

  // To display the charging status, you must first discharge the battery,
  // and it is impossible to read the full charge when it is fully charged
  if (axp.isChargeing()) {
    Serial.print("Charge:");
    Serial.print(axp.getBattChargeCurrent());
    Serial.println(" mA");
  } else {
    // Show current consumption
    Serial.print("Discharge:");
    Serial.print(axp.getBattDischargeCurrent());
    Serial.println(" mA");

    /*getBattPercentage just only support axp202 */
    if (false) {
      Serial.print("Per: ");
      Serial.print(axp.getBattPercentage());
      Serial.println(" %");
    }
  }
}

void setup() {
  Serial.begin(115200);
  // 不delay会有连不上i2c问题
  delay(100);
  Wire.begin(i2c_sda, i2c_scl);

  //! Use the Wire port
  int ret = axp.begin(Wire, AXP192_SLAVE_ADDRESS);
  // int ret = axp.begin(Wire);
  if (ret == AXP_FAIL) {
    Serial.println("AXP Power begin failed");
    while (1);
  }

  Serial.println();
  axp.setVWarningLevel1(3450);
  axp.setVWarningLevel2(3400);
  axp.setPowerDownVoltage(2600);

  uint16_t level1 = axp.getVWarningLevel1();
  uint16_t level2 = axp.getVWarningLevel2();
  Serial.printf("getVWarningLevel1:%u mV \n", level1);
  Serial.printf("getVWarningLevel2:%u mV \n", level2);
  Serial.printf("getPowerDonwVoltage:%u mV \n", axp.getPowerDownVoltage());
  Serial.printf("getPowerDonwVoltage:%u mV \n", axp.getPowerDownVoltage());
  Serial.println();

  //! attachInterrupt to gpio 35
  pinMode(axp_irq_pin, INPUT_PULLUP);
  attachInterrupt(axp_irq_pin, setFlag, FALLING);
  axp.clearIRQ();

  //! enable all irq channel
  axp.enableIRQ(AXP202_ALL_IRQ, true);

  axp.setPowerOutPut(AXP202_DCDC3, AXP202_ON);
  axp.setPowerOutPut(AXP202_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP202_LDO2,  AXP202_ON);
  // axp.setPowerOutPut(AXP202_LDO4,  AXP202_ON);
  axp.setPowerOutPut(AXP202_DCDC2, AXP202_ON);
  axp.setDCDC2Voltage(2275);
  // axp.setLDO4Voltage(AXP202_LDO4_3300MV);
  axp.setLDO3Voltage(3500);
  axp.setPowerOutPut(AXP202_LDO3, AXP202_ON);

  axp.adc1Enable(
    AXP202_VBUS_VOL_ADC1 |
    AXP202_VBUS_CUR_ADC1 |
    AXP202_ACIN_VOL_ADC1 |
    AXP202_ACIN_CUR_ADC1 |
    AXP202_BATT_CUR_ADC1 |
    AXP202_APS_VOL_ADC1 |
    AXP202_BATT_VOL_ADC1,
    true);

  printAXPStatus();
  // axp.setTimer(1);
}

void readIRQ() {
  axp.readIRQ();
  Serial.println("axp20x irq enter!");
  if (axp.isAcinOverVoltageIRQ()) {
    Serial.printf("isAcinOverVoltageIRQ\n");
  }
  if (axp.isAcinPlugInIRQ()) {
    Serial.printf("isAcinPlugInIRQ\n");
  }
  if (axp.isAcinRemoveIRQ()) {
    Serial.printf("isAcinRemoveIRQ\n");
  }
  if (axp.isVbusOverVoltageIRQ()) {
    Serial.printf("isVbusOverVoltageIRQ\n");
  }
  if (axp.isVbusPlugInIRQ()) {
    Serial.printf("isVbusPlugInIRQ\n");
  }
  if (axp.isVbusRemoveIRQ()) {
    Serial.printf("isVbusRemoveIRQ\n");
  }
  if (axp.isVbusLowVHOLDIRQ()) {
    Serial.printf("isVbusLowVHOLDIRQ\n");
  }
  if (axp.isBattPlugInIRQ()) {
    Serial.printf("isBattPlugInIRQ\n");
  }
  if (axp.isBattRemoveIRQ()) {
    Serial.printf("isBattRemoveIRQ\n");
  }
  if (axp.isBattEnterActivateIRQ()) {
    Serial.printf("isBattEnterActivateIRQ\n");
  }
  if (axp.isBattExitActivateIRQ()) {
    Serial.printf("isBattExitActivateIRQ\n");
  }
  if (axp.isChargingIRQ()) {
    Serial.printf("isChargingIRQ\n");
  }
  if (axp.isChargingDoneIRQ()) {
    Serial.printf("isChargingDoneIRQ\n");
  }
  if (axp.isBattTempLowIRQ()) {
    Serial.printf("isBattTempLowIRQ\n");
  }
  if (axp.isBattTempHighIRQ()) {
    Serial.printf("isBattTempHighIRQ\n");
  }
  if (axp.isChipOvertemperatureIRQ()) {
    Serial.printf("isChipOvertemperatureIRQ\n");
  }
  if (axp.isChargingCurrentLessIRQ()) {
    Serial.printf("isChargingCurrentLessIRQ\n");
  }
  if (axp.isDC2VoltageLessIRQ()) {
    Serial.printf("isDC2VoltageLessIRQ\n");
  }
  if (axp.isDC3VoltageLessIRQ()) {
    Serial.printf("isDC3VoltageLessIRQ\n");
  }
  if (axp.isLDO3VoltageLessIRQ()) {
    Serial.printf("isLDO3VoltageLessIRQ\n");
  }
  if (axp.isPEKShortPressIRQ()) {
    Serial.printf("isPEKShortPressIRQ\n");
  }
  if (axp.isPEKLongtPressIRQ()) {
    Serial.printf("isPEKLongtPressIRQ\n");
  }
  if (axp.isNOEPowerOnIRQ()) {
    Serial.printf("isNOEPowerOnIRQ\n");
  }
  if (axp.isNOEPowerDownIRQ()) {
    Serial.printf("isNOEPowerDownIRQ\n");
  }
  if (axp.isVBUSEffectiveIRQ()) {
    Serial.printf("isVBUSEffectiveIRQ\n");
  }
  if (axp.isVBUSInvalidIRQ()) {
    Serial.printf("isVBUSInvalidIRQ\n");
  }
  if (axp.isVUBSSessionIRQ()) {
    Serial.printf("isVUBSSessionIRQ\n");
  }
  if (axp.isVUBSSessionEndIRQ()) {
    Serial.printf("isVUBSSessionEndIRQ\n");
  }
  if (axp.isLowVoltageLevel1IRQ()) {
    Serial.printf("isLowVoltageLevel1IRQ\n");
  }
  if (axp.isLowVoltageLevel2IRQ()) {
    Serial.printf("isLowVoltageLevel2IRQ\n");
  }
  if (axp.isTimerTimeoutIRQ()) {
    Serial.printf("isTimerTimeoutIRQ\n");
    axp.offTimer();
    axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
  }
  if (axp.isPEKRisingEdgeIRQ()) {
    Serial.printf("isPEKRisingEdgeIRQ\n");
  }
  if (axp.isPEKFallingEdgeIRQ()) {
    Serial.printf("isPEKFallingEdgeIRQ\n");
  }
  if (axp.isGPIO3InputEdgeTriggerIRQ()) {
    Serial.printf("isGPIO3InputEdgeTriggerIRQ\n");
  }
  if (axp.isGPIO2InputEdgeTriggerIRQ()) {
    Serial.printf("isGPIO2InputEdgeTriggerIRQ\n");
  }
  if (axp.isGPIO1InputEdgeTriggerIRQ()) {
    Serial.printf("isGPIO1InputEdgeTriggerIRQ\n");
  }
  if (axp.isGPIO0InputEdgeTriggerIRQ()) {
    Serial.printf("isGPIO0InputEdgeTriggerIRQ\n");
  }
}

void loop() {
  if (axpIrq) {
    axpIrq = 0;
    readIRQ();
    axp.clearIRQ();
  } else {
    printAXPStatus();
    delay(100);
  }
}
