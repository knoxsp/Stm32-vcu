#ifndef MAZDA_RX8_H
#define MAZDA_RX8_H

/*  This library supports the CAN messages for the MAZDA RX8 for driving dash gauges, putting out malf lights etc

*/

#include <stdint.h>
#include <vehicle.h>
#include "utils.h"
#include "my_math.h"
#include "digio.h"
#include "stm32_can.h"

class MAZDA_RX8 : public Vehicle
{
public:
   void SetCanInterface(CanHardware*);
   void SetRevCounter(int speed) { revCounter = speed; }
   void SetTemperatureGauge(float temp) { temperature = temp; }
   bool Ready();
   bool Start();
   void Task100Ms();
   void DecodeCAN(int, uint32_t* data);
   void handleImmobiliserMsg(uint32_t data[2]);
   void handleWheelSpeedMsg(uint32_t data[2]);
   void SetFuelGauge(float level);
   void updateDSC();
   void updateMIL();
   void updatePCM();
   void sendTractionControlMessages();

private:
   bool checkEngineMIL;
   bool checkEngineBL;
   uint8_t engTemp;
   uint8_t odo;
   bool oilPressure;
   bool lowWaterMIL;
   bool batChargeMIL;
   bool oilPressureMIL;
   uint16_t engineRPM;
   uint16_t throttlePedal;
   uint16_t vehicleSpeed;
   bool dscOff;
   bool absMIL;
   bool brakeFailMIL;
   bool etcActiveBL;
   bool etcDisabled;
   uint16_t frontLeft;
   uint16_t frontRight;
   uint16_t rearLeft;
   uint16_t rearRight;
   int revCounter;
   float temperature;
};

#endif /* MAZDA_RX8_H */

